from PySide6.QtNetwork import QTcpSocket, QTcpServer, QHostAddress, QUdpSocket
from PySide6.QtCore import QObject, Signal as pyqtSignal, QThread, QTimer
from struct import unpack
from collections import deque
import numpy as np
from numpy import array, zeros, cos, sin, pi
from json import dumps, loads
from numpy.linalg import inv
from pseudo_intersection import euler_to_rotation
from datetime import datetime
from traceback import print_exc
from log import log_command_send, log_command_recv, log_realtime_send, log_wmps_raw

wMPSTcpAddress = "192.168.1.201"  # TCP服务器地址
lastPredictTimeCount = 0
PREAVERAGE_COORDINATE_WINDOW = 5
MOVING_AVERAGE_COORDINATE_WINDOW = 5




def circular_mean_phase(values):
    """Mean wrapped phase values in turns, preserving the 0/1 boundary."""
    values = np.asarray(values, dtype=float)
    angles = values * 2 * pi
    sin_mean = float(np.mean(np.sin(angles)))
    cos_mean = float(np.mean(np.cos(angles)))
    if abs(sin_mean) < 1e-12 and abs(cos_mean) < 1e-12:
        return float(np.mean(values) % 1.0)
    return float((np.arctan2(sin_mean, cos_mean) / (2 * pi)) % 1.0)


class wMPSProcessorConnection(QObject):
    """处理与wMPS处理器的TCP连接和数据接收"""
    rawArrived = pyqtSignal(int, object)
    connection_status_changed = pyqtSignal(bool, object)  # 添加连接状态信号，参数为是否连接成功和设备信息

    def __init__(self):
        super(wMPSProcessorConnection, self).__init__()
        self.m_tcpServer = QTcpServer(self)
        self.m_tcpServer.newConnection.connect(self.new_socket_slot)
        self.m_processor_connection = dict()
        self.m_timeCountRecord = dict()
        self.status_reported = False
        self.m_socket_buffers = dict()
        print(f"TCP服务器启动: 监听地址 {wMPSTcpAddress}:8234")

    def check_connection_status(self):
        """检查并报告连接状态"""
        # 如果已经报告过状态，直接返回
        if self.status_reported:
            return
        # 延迟2秒后检查，给设备连接时间
        QTimer.singleShot(2000, self._check_and_report_status)

    def _check_and_report_status(self):
        """实际执行连接状态检查和报告"""
        # 如果已经报告过状态，直接返回
        if self.status_reported:
            return

        # 检查是否有设备连接
        if self.m_processor_connection:
            # 构建成功连接状态信息 - 简化版，不包含设备数量信息
            status_data = {
                "InstrumentName": "wMPS",
                "OperationID": "wMPSConnected",
                "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }

            # 发出连接成功信号
            self.connection_status_changed.emit(True, status_data)
            print(f"wMPS连接成功: {dumps(status_data)}")
            self.status_reported = True  # 标记为已报告状态
        else:
            # 没有设备连接，报告连接失败
            status_data = {
                "InstrumentName": "wMPS",
                "OperationID": "wMPSConnectFailed",
                "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }

            # 发出连接失败信号
            self.connection_status_changed.emit(False, status_data)
            print(f"wMPS连接失败: {dumps(status_data)}")

    def new_socket_slot(self):
        """处理新的TCP连接"""
        print(f"检测到新连接请求")
        socket = self.m_tcpServer.nextPendingConnection()
        peer_address = socket.peerAddress().toString()
        peer_port = socket.peerPort()
        self.m_processor_connection[peer_address] = socket
        print('Processor connected with address {}, port {}'.format(peer_address, str(peer_port)))
        socket.readyRead.connect(self.read_processor_data)

        # 有新设备连接，但只在第一次连接时检查状态
        if not self.status_reported:
            QTimer.singleShot(1000, self._check_and_report_status)  # 延迟2秒后检查状态

    def reinitialize_server(self, new_address, port=8234):
        """重新初始化TCP服务器"""
        print(f"尝试重新初始化TCP服务器: {new_address}:{port}")  # 打印实际使用的端口值

        # 关闭现有服务器
        if self.m_tcpServer.isListening():
            print(f"关闭现有TCP服务器")
            self.m_tcpServer.close()

        # 清空现有连接
        for addr in list(self.m_processor_connection.keys()):
            if self.m_processor_connection[addr].state() == QTcpSocket.ConnectedState:
                print(f"关闭与 {addr} 的连接")
                self.m_processor_connection[addr].close()
        self.m_processor_connection.clear()

        # 重置状态报告标志，允许在服务器重新初始化后再次报告
        self.status_reported = False

        # 重新启动服务器
        success = self.m_tcpServer.listen(QHostAddress(new_address), port)
        print(f"TCP服务器重新监听: {new_address}:{port}, 成功: {success}")

        # 检查并报告重新连接后的状态
        QTimer.singleShot(3000, self._check_and_report_status)  # 延迟3秒后检查状态

        return success

    def read_processor_data(self):
        """读取处理器发送的数据 (修复了粘包和半包问题)"""
        for addr in list(self.m_processor_connection.keys()):
            sock = self.m_processor_connection[addr]
            if self.sender() is sock:
                try:
                    # 1. 确保该地址有缓存字典
                    if addr not in self.m_socket_buffers:
                        self.m_socket_buffers[addr] = bytes()

                    # 2. 读取当前网卡缓冲区的所有数据，并追加到持久化缓存中
                    chunk = sock.readAll().data()
                    if not chunk:
                        continue
                    self.m_socket_buffers[addr] += chunk

                    buffer = self.m_socket_buffers[addr]

                    # 3. 开始滑动窗口解包
                    while len(buffer) >= 9:  # 至少需要9字节才能解析头部
                        # 查找包头 0xAA
                        head_idx = buffer.find(b'\xAA')

                        if head_idx == -1:
                            # 没找到包头，说明全是脏数据，直接清空并跳出
                            buffer = bytes()
                            break
                        elif head_idx > 0:
                            # 包头不在第一位，说明包头前面有残缺的脏数据，丢弃它们
                            buffer = buffer[head_idx:]
                            continue

                        # 此时 buffer[0] 绝对是 0xAA，解析前9个字节的头部
                        header, length, device_type, id_number, fcode = unpack('>BHBIB', buffer[:9])

                        # 完整的包长 = 包头(1) + 长度字段(2) + length
                        packet_len = 3 + length

                        # 检查当前缓存是否够一个完整的包
                        if len(buffer) < packet_len:
                            # 数据不够（半包现象），跳出 while 循环，等待下一次 readyRead 触发接收剩余数据
                            break

                        # 数据长度足够，校验包尾是否为 5A 5A
                        end_pos = packet_len - 2
                        if buffer[end_pos:packet_len] == b'\x5A\x5A':
                            # 提取有效负载 (去掉前9字节和后2字节)
                            payload = buffer[9:end_pos]

                            try:
                                # 解析有效通道数据
                                raw_data = self.parse_data(id_number, payload)
                                if raw_data:
                                    self.rawArrived.emit(id_number, raw_data)
                            except Exception as e:
                                print(f"解析有效负载失败: {str(e)}")

                        else:
                            # 长度够了但包尾不对，说明这是一个伪造的包头 0xAA 或是错位了
                            print(f"包尾校验失败，丢弃错位包头")
                            # 强制舍弃当前的 0xAA 字节，下一轮循环会去寻找下一个 0xAA
                            buffer = buffer[1:]
                            continue

                        # 成功处理完一个包，将游标向后移动，切掉已处理的部分
                        buffer = buffer[packet_len:]

                    # 4. 把剩下未处理完的（半包）数据存回字典，留给下次使用
                    self.m_socket_buffers[addr] = buffer

                except Exception as e:
                    print(f"读取或处理数据时发生异常: {str(e)}")

    def parse_data(self, number_id, payload):
        """解析通道数据 (改用游标偏移法，更稳定)"""
        raw_data = dict()
        offset = 0
        payload_len = len(payload)

        try:
            # 只要剩下的数据够解析头部 (通道号1字节 + 数据组数1字节 = 2字节)
            while offset + 2 <= payload_len:
                channel, data_count = unpack('>BB', payload[offset:offset + 2])
                offset += 2

                if channel not in raw_data:
                    raw_data[channel] = dict()

                # 解析该通道下的每组发射站数据
                for _ in range(data_count):
                    # 检查剩下长度是否够一组数据(18字节)
                    if offset + 18 <= payload_len:
                        speed, phase1, phase2, period, sync_tag = unpack('>hffII', payload[offset:offset + 18])
                        offset += 18

                        # 处理相位反转逻辑
                        if 0 < phase2 - phase1 < 0.5:
                            tmp = phase1
                            phase1 = 1 - phase2
                            phase2 = 1 - tmp
                        else:
                            phase1 = 1 - phase1
                            phase2 = 1 - phase2

                        # 过滤重复的同步帧数据
                        nameTag = f"{number_id}@{channel}@{speed}"
                        if nameTag not in self.m_timeCountRecord or self.m_timeCountRecord[nameTag] != sync_tag:
                            raw_data[channel][speed] = [phase1, phase2, period, sync_tag]

                        self.m_timeCountRecord[nameTag] = sync_tag
                    else:
                        # 负载长度异常截断，直接跳出防止崩溃
                        break
        except Exception as e:
            print(f"内部通道解析异常: {e}")

        return raw_data

    def get_connected_processors(self):
        """返回当前连接的处理器地址列表"""
        return list(self.m_processor_connection.keys())

class rawResolveThread(QObject):
    """处理原始数据并计算3D坐标"""
    coordArrived = pyqtSignal(int, object)

    def __init__(self):
        super(rawResolveThread, self).__init__()
        self.RPM_2_trans = dict()  # 转速到发射器ID的映射
        self.trans_2_RPM = dict()  # 发射器ID到转速的映射
        self.m_innPara = dict()  # 内部参数
        self.m_extPara = dict()  # 外部参数
        self.preaverage_coordinate_window = PREAVERAGE_COORDINATE_WINDOW
        self.moving_average_coordinate_window = MOVING_AVERAGE_COORDINATE_WINDOW
        self._preaverage_coordinate_buffers = dict()
        self._moving_average_coordinate_buffers = dict()
        print("初始化rawResolveThread，等待用户加载内外参文件")

    def _reset_coordinate_filters(self):
        self._preaverage_coordinate_buffers.clear()
        self._moving_average_coordinate_buffers.clear()

    def read_innpara_from_txt(self, file_path):
        """从文件读取内部参数"""
        try:
            self.m_innPara.clear()  # 清空旧参数
            self.RPM_2_trans.clear()
            self.trans_2_RPM.clear()
            self._reset_coordinate_filters()

            print(f"开始从 {file_path} 读取内参")
            with open(file_path) as f:
                str_lists = f.readlines()
                loaded_count = 0
                for line in str_lists:
                    innLine = line.split()
                    if innLine.__len__() == 10:
                        self.RPM_2_trans[int(innLine[1])] = int(innLine[0])
                        self.trans_2_RPM[int(innLine[0])] = int(innLine[1])
                        self.m_innPara[int(innLine[0])] = array(innLine[2:10], 'float')
                        loaded_count += 1

            print(f"内参加载成功: 共加载 {loaded_count} 组参数")
            return True
        except Exception as e:
            print(f"读取内部参数文件失败: {str(e)}")
            print_exc()
            return False

    def read_extpara_from_txt(self, file_path):
        """从文件读取外部参数"""
        try:
            self.m_extPara.clear()  # 清空旧参数
            self._reset_coordinate_filters()

            print(f"开始从 {file_path} 读取外参")
            with open(file_path) as f:
                str_lists = f.readlines()
                loaded_count = 0
                for line in str_lists:
                    innLine = line.split()
                    if innLine.__len__() == 7:
                        out_para = array(innLine[1:7], 'float')
                        r = euler_to_rotation(out_para[0], out_para[1], out_para[2])
                        t = array([[out_para[3], out_para[4], out_para[5]]])
                        self.m_extPara[int(innLine[0])] = (r, t)
                        loaded_count += 1

            print(f"外参加载成功: 共加载 {loaded_count} 组参数")
            return True
        except Exception as e:
            print(f"读取外部参数文件失败: {str(e)}")
            print_exc()
            return False

    def preaverage_coordinate(self, processor_ID, rawDict):
        """Pre-average raw scan phases before coordinate solving."""
        averaged_raw = dict()
        for channel_index, transmitter_dict in rawDict.items():
            averaged_raw[channel_index] = dict()
            for transmitter_rpm, values in transmitter_dict.items():
                sample = np.asarray(values, dtype=float).reshape(-1)
                if sample.size < 2 or not np.all(np.isfinite(sample[:2])):
                    averaged_raw[channel_index][transmitter_rpm] = values
                    continue

                key = (int(processor_ID), int(channel_index), int(transmitter_rpm))
                buf = self._preaverage_coordinate_buffers.get(key)
                if buf is None:
                    buf = deque(maxlen=self.preaverage_coordinate_window)
                    self._preaverage_coordinate_buffers[key] = buf
                buf.append(sample.copy())

                samples = np.vstack(buf)
                averaged = sample.copy()
                averaged[0] = circular_mean_phase(samples[:, 0])
                averaged[1] = circular_mean_phase(samples[:, 1])
                if averaged.size > 2:
                    averaged[2] = float(np.mean(samples[:, 2]))
                averaged_raw[channel_index][transmitter_rpm] = averaged.tolist()
        return averaged_raw

    def moving_average_coordinate(self, processor_ID, channel_index, coordinate):
        """Post-average solved XYZ coordinates with a fixed frame window."""
        coord = np.asarray(coordinate, dtype=float).reshape(-1)
        if coord.size < 3 or not np.all(np.isfinite(coord[:3])):
            return coordinate

        key = (int(processor_ID), int(channel_index))
        buf = self._moving_average_coordinate_buffers.get(key)
        if buf is None:
            buf = deque(maxlen=self.moving_average_coordinate_window)
            self._moving_average_coordinate_buffers[key] = buf
        buf.append(coord[:3].copy())
        return np.mean(np.vstack(buf), axis=0)

    def calculate_Coord(self, processor_ID, rawDict):
        """Calculate coordinates with the same solver as LaserTracer V2.2."""
        full_dict = dict()
        coordinate_dict = dict()
        remove_channel = set()
        rawDict = self.preaverage_coordinate(processor_ID, rawDict)

        for channel_index in rawDict:
            validStation = 0
            for transmitter_RPM in rawDict[channel_index]:
                if transmitter_RPM in self.RPM_2_trans and self.RPM_2_trans[transmitter_RPM] in self.m_extPara:
                    validStation += 1
            if validStation < 2:
                continue

            a = zeros((2 * rawDict[channel_index].__len__(), 3))
            b = zeros((2 * rawDict[channel_index].__len__(), 1))
            i = 0

            for transmitter_RPM in rawDict[channel_index]:
                if transmitter_RPM not in self.RPM_2_trans or self.RPM_2_trans[transmitter_RPM] not in self.m_extPara:
                    continue

                transID = self.RPM_2_trans[transmitter_RPM]
                inpara = self.m_innPara[transID]
                rotation = self.m_extPara[transID][0]
                transformation = self.m_extPara[transID][1].T
                scan_time = rawDict[channel_index][transmitter_RPM][0:2]

                ct1 = cos(scan_time[0] * 2 * pi)
                st1 = sin(scan_time[0] * 2 * pi)
                a[2 * i, :] = inpara[0:3].dot([[ct1, -st1, 0],
                                               [st1, ct1, 0],
                                               [0, 0, 1]]).dot(rotation)
                b[2 * i] = -inpara[3] - a[2 * i, :].dot(rotation.T).dot(transformation)

                ct2 = cos(scan_time[1] * 2 * pi)
                st2 = sin(scan_time[1] * 2 * pi)
                a[2 * i + 1, :] = inpara[4:7].dot([[ct2, -st2, 0],
                                                   [st2, ct2, 0],
                                                   [0, 0, 1]]).dot(rotation)
                b[2 * i + 1] = -inpara[7] - a[2 * i + 1, :].dot(rotation.T).dot(transformation)

                i = i + 1

            try:
                coordinate = inv(a.T.dot(a)).dot(a.T.dot(b)).T
            except Exception as exc:
                print(f"[wMPS solver] processor={processor_ID}, channel={channel_index}: solve failed: {exc}")
                continue

            residual = float(max(abs(a.dot(coordinate.T) - b)))
            if residual > 1:
                remove_channel.add(channel_index)
                print(
                    f"[wMPS solver] processor={processor_ID}, channel={channel_index}: "
                    f"large residual={residual:.6f}, coord={coordinate[0]}"
                )
            else:
                coordinate_dict[channel_index] = self.moving_average_coordinate(
                    processor_ID, channel_index, coordinate[0]
                )

        full_dict['coord'] = coordinate_dict
        full_dict['raw'] = dict()

        for channel in rawDict:
            if channel in remove_channel:
                continue
            if channel not in full_dict['raw']:
                full_dict['raw'][channel] = dict()
            for transmitter_RPM in rawDict[channel]:
                if transmitter_RPM not in self.RPM_2_trans:
                    continue
                trans_id = self.RPM_2_trans[transmitter_RPM]
                full_dict['raw'][channel][trans_id] = rawDict[channel][transmitter_RPM]

        full_dict['all_raw'] = rawDict
        self.coordArrived.emit(processor_ID, full_dict)

class CommandClient(QObject):
    """处理常规命令通信的客户端"""
    response_received = pyqtSignal(str)  # 接收到响应的信号

    def __init__(self, server_ip="127.0.0.1", server_port=8235):
        super(CommandClient, self).__init__()
        self.m_tcpSocket = QTcpSocket(self)
        self.server_ip = server_ip
        self.server_port = server_port
        self.m_tcpSocket.readyRead.connect(self.read_server_response)
        self.connected = False
        print(f"命令通信客户端已创建: 目标服务器 {server_ip}:{server_port}")

    def connect_to_server(self):
        """连接到服务器"""
        if self.m_tcpSocket.state() == QTcpSocket.ConnectedState:
            return True

        self.m_tcpSocket.connectToHost(self.server_ip, self.server_port)
        if self.m_tcpSocket.waitForConnected(3000):  # 等待3秒连接
            self.connected = True
            print(f"命令通信客户端已连接到服务器: {self.server_ip}:{self.server_port}")

            # 连接成功后发送身份识别包
            self.send_identity_message()

            return True
        else:
            self.connected = False
            print(f"命令通信客户端连接服务器失败: {self.server_ip}:{self.server_port}")
            return False

    def send_identity_message(self):
        """发送身份识别信息"""

        identity_data = {
            "InstrumentName": "wMPS",
            "OperationID": "IsCommonClient",
            "TimeStamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

        message = dumps(identity_data)
        self.send_message(message)
        print(f"已发送命令客户端身份识别信息: {message}")

    def disconnect_from_server(self):
        """断开与服务器的连接"""
        if self.m_tcpSocket.state() == QTcpSocket.ConnectedState:
            self.m_tcpSocket.disconnectFromHost()
            self.connected = False
            print(f"命令通信客户端断开连接: {self.server_ip}:{self.server_port}")
            return True
        return False

    def read_server_response(self):
        """读取服务器响应"""
        try:
            data = self.m_tcpSocket.readAll().data().decode('utf-8')
            if data:
                print(f"收到服务器响应: {data[:100]}...")
                log_command_recv(data)
                self.response_received.emit(data)
        except Exception as e:
            print(f"读取服务器响应出错: {str(e)}")

    def send_message(self, message):
        """向服务器发送消息"""
        if not self.connected:
            success = self.connect_to_server()
            if not success:
                print(f"无法连接到服务器 {self.server_ip}:{self.server_port}，消息未发送")
                return False

        try:
            log_command_send(message)
            self.m_tcpSocket.write(message.encode('utf-8'))
            self.m_tcpSocket.flush()
            print(f"向服务器 {self.server_ip}:{self.server_port} 发送消息: {message[:100]}...")
            return True
        except Exception as e:
            print(f"发送消息出错: {str(e)}")
            return False

    def update_server_address(self, new_ip, new_port):
        """更新服务器地址"""
        # 如果已连接，先断开
        if self.connected:
            self.disconnect_from_server()

        self.server_ip = new_ip
        self.server_port = new_port
        print(f"命令通信客户端更新服务器地址: {self.server_ip}:{self.server_port}")
        return True

class RealtimeDataClient(QObject):
    """处理实时数据通信的客户端"""
    response_received = pyqtSignal(str)  # 接收到响应的信号

    def __init__(self, server_ip="127.0.0.1", server_port=8236):
        super(RealtimeDataClient, self).__init__()
        self.m_tcpSocket = QTcpSocket(self)
        self.server_ip = server_ip
        self.server_port = server_port
        self.m_tcpSocket.readyRead.connect(self.read_server_response)
        self.connected = False

        print(f"实时数据客户端已创建: 目标服务器 {server_ip}:{server_port}")

    def connect_to_server(self):
        """连接到服务器"""
        if self.m_tcpSocket.state() == QTcpSocket.ConnectedState:
            return True

        self.m_tcpSocket.connectToHost(self.server_ip, self.server_port)
        if self.m_tcpSocket.waitForConnected(3000):  # 等待3秒连接
            self.connected = True
            print(f"实时数据客户端已连接到服务器: {self.server_ip}:{self.server_port}")

            # 连接成功后发送身份识别包
            self.send_identity_message()

            return True
        else:
            self.connected = False
            print(f"实时数据客户端连接服务器失败: {self.server_ip}:{self.server_port}")
            return False

    def send_identity_message(self):
        """发送身份识别信息"""
        identity_data = {
            "InstrumentName": "wMPS",
            "OperationID": "IsRealTimeClient",
            "TimeStamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

        message = dumps(identity_data)
        self.send_message(message)
        print(f"已发送实时数据客户端身份识别信息: {message}")

    def disconnect_from_server(self):
        """断开与服务器的连接"""
        if self.m_tcpSocket.state() == QTcpSocket.ConnectedState:
            self.m_tcpSocket.disconnectFromHost()
            self.connected = False
            print(f"实时数据客户端断开连接: {self.server_ip}:{self.server_port}")
            return True
        return False

    def read_server_response(self):
        """读取服务器响应"""
        try:
            data = self.m_tcpSocket.readAll().data().decode('utf-8')
            if data:
                print(f"收到服务器响应: {data[:100]}...")
                self.response_received.emit(data)
        except Exception as e:
            print(f"读取服务器响应出错: {str(e)}")

    def send_message(self, message):
        """向服务器发送消息"""
        if not self.connected:
            success = self.connect_to_server()
            if not success:
                print(f"无法连接到服务器 {self.server_ip}:{self.server_port}，消息未发送")
                return False

        try:
            log_realtime_send(message)
            self.m_tcpSocket.write(message.encode('utf-8'))
            self.m_tcpSocket.flush()
            print(f"向服务器 {self.server_ip}:{self.server_port} 发送消息: {message[:100]}...")
            return True
        except Exception as e:
            print(f"发送消息出错: {str(e)}")
            return False

    def update_server_address(self, new_ip, new_port):
        """更新服务器地址"""
        # 如果已连接，先断开
        if self.connected:
            self.disconnect_from_server()

        self.server_ip = new_ip
        self.server_port = new_port
        print(f"实时数据客户端更新服务器地址: {self.server_ip}:{self.server_port}")
        return True


class UdpBroadcaster(QObject):
    """UDP广播模块，用于发送解算后的坐标数据"""

    def __init__(self, port=10000):
        super(UdpBroadcaster, self).__init__()
        self.udp_socket = QUdpSocket(self)
        self.broadcast_port = port
        self.broadcast_timer = QTimer(self)
        self.broadcast_timer.timeout.connect(self.broadcast_data)
        self.processor_data = {}  # 存储每个处理器的最新数据
        self.last_broadcast_time = datetime.now()
        print(f"UDP发送模块初始化，目标: 127.0.0.1:{port}")

    def start_broadcasting(self, interval=100):
        """启动广播，interval为广播间隔（毫秒）"""
        if not self.broadcast_timer.isActive():
            self.broadcast_timer.start(interval)
            print(f"UDP发送已启动，间隔: {interval}ms，目标: 127.0.0.1:{self.broadcast_port}")

    def stop_broadcasting(self):
        """停止广播"""
        if self.broadcast_timer.isActive():
            self.broadcast_timer.stop()
            print("UDP发送已停止")

    def update_data(self, processor_coordinates, receiver_status=None):
        """更新要广播的坐标数据"""
        if not processor_coordinates:
            return

        # 获取所有处理器ID
        processor_ids = set()
        for receiver_id in processor_coordinates:
            processor_id = int(receiver_id.split('@')[0])
            processor_ids.add(processor_id)

        # 为每个处理器构建数据包
        for processor_id in processor_ids:
            # 构建与原始格式相同的数据结构
            coord_data = {}

            # 收集特定处理器的所有通道数据
            for receiver_id, coord_values in processor_coordinates.items():
                if receiver_id.startswith(f"{processor_id}@"):
                    # 提取通道ID
                    channel_id = receiver_id.split('@')[1]

                    # 检查坐标有效性
                    valid = True
                    if receiver_status and receiver_id in receiver_status:
                        valid = (receiver_status[receiver_id] == 1)  # 1表示正常状态

                    # 获取发射站信号
                    transmitter_count = ""
                    if hasattr(self, 'processor_raw') and processor_id in self.processor_raw and int(channel_id) in \
                            self.processor_raw[processor_id]:
                        trans_ids = list(self.processor_raw[processor_id][int(channel_id)].keys())
                        transmitter_count = ".".join(map(str, trans_ids)) + "."

                    # 添加到坐标数据字典
                    coord_data[channel_id] = {
                        "Flags": 0,
                        "TransCount": transmitter_count,
                        "Valid": valid,
                        "X": float(coord_values[0]),
                        "Y": float(coord_values[1]),
                        "Z": float(coord_values[2])
                    }

            # 只有在有数据时才更新处理器数据
            if coord_data:
                # 构建完整数据包
                self.processor_data[processor_id] = {
                    "coord": coord_data,
                    "id": processor_id,
                }

        self.last_broadcast_time = datetime.now()

    def set_processor_raw(self, processor_raw):
        """设置原始处理器数据，用于提取发射站ID"""
        self.processor_raw = processor_raw

    def broadcast_data(self):
        """发送数据到本地127.0.0.1:10000"""
        # 如果没有数据，不发送
        if not self.processor_data:
            return

        try:
            # 为每个处理器发送独立的数据包
            for processor_id, data in self.processor_data.items():
                # 将数据转换为JSON字符串
                json_data = dumps(data)

                # 转换为字节并发送到本地127.0.0.1
                data_bytes = json_data.encode('utf-8')
                bytes_sent = self.udp_socket.writeDatagram(
                    data_bytes,
                    QHostAddress("127.0.0.1"),  # 明确指定本地回环地址
                    self.broadcast_port
                )

                # 输出调试信息
                current_time = datetime.now()
                delta = (current_time - self.last_broadcast_time).total_seconds()
                print(f"UDP发送处理器ID {processor_id} 数据到 127.0.0.1:{self.broadcast_port}: {len(data_bytes)}字节")

            # 更新最后发送时间
            self.last_broadcast_time = datetime.now()

        except Exception as e:
            print(f"UDP发送出错: {str(e)}")

class UdpReceiver(QObject):
    """UDP数据接收器，接收外部发来的已解算坐标数据"""
    data_received = pyqtSignal(int, object)

    def __init__(self, ip="127.0.0.1", port=10000, forward_port=10001):
        super(UdpReceiver, self).__init__()
        self.ip = ip
        self.port = port
        self.forward_port = forward_port
        self.socket = QUdpSocket(self)
        self.forward_socket = QUdpSocket(self)  # 专用于转发的socket
        print(f"创建UDP接收器，监听地址: {self.ip}:{self.port}，转发至 127.0.0.1:{self.forward_port}")

    def start_listening(self):
        """开始监听UDP数据"""
        from PySide6.QtNetwork import QHostAddress
        success = self.socket.bind(QHostAddress(self.ip), self.port)
        if success:
            self.socket.readyRead.connect(self.process_pending_datagrams)
            print(f"UDP接收器已开始监听 {self.ip}:{self.port}")
        else:
            print(f"UDP接收器绑定 {self.ip}:{self.port} 失败!")
        return success

    def stop_listening(self):
        """停止监听UDP数据"""
        try:
            self.socket.readyRead.disconnect()
        except:
            pass
        if self.socket.state() != QUdpSocket.UnconnectedState:
            self.socket.close()
        print(f"UDP接收器已停止监听 {self.ip}:{self.port}")

    def process_pending_datagrams(self):
        """处理待接收的数据包"""
        while self.socket.hasPendingDatagrams():
            datagram, host, port = self.socket.readDatagram(self.socket.pendingDatagramSize())
            if datagram:
                # 原始数据原封不动转发到 127.0.0.1:10001
                self.forward_socket.writeDatagram(
                    datagram,
                    QHostAddress("127.0.0.1"),
                    self.forward_port
                )
                print(f"[UDP转发] 已将 {len(datagram)} 字节转发到 127.0.0.1:{self.forward_port}")
                self.process_data(datagram)

    def process_data(self, data):
        """解析收到的UDP数据并发出信号"""
        try:
            data_str = bytes(data).decode('utf-8')
            json_data = loads(data_str)

            processor_id = json_data.get("id", 0)

            # 处理 ID=590593573 的点位测量数据包
            if processor_id == 590593573:
                coord_dict = json_data.get("coord", {})
                # 优先读取 -1 通道，没有则读取 0 通道
                channel_data = coord_dict.get("-1") or coord_dict.get(-1) or coord_dict.get("0") or coord_dict.get(0)
                # 找出实际通道key
                channel_key = None
                for k in ["-1", -1, "0", 0]:
                    if coord_dict.get(k):
                        channel_key = k
                        break
                if channel_data and channel_key is not None:
                    x = channel_data.get("X", 0.0)
                    y = channel_data.get("Y", 0.0)
                    z = channel_data.get("Z", 0.0)
                    channel_int = int(channel_key)
                    self.data_received.emit(processor_id, {
                        "point": {"X": x, "Y": y, "Z": z},
                        "channel": channel_int,
                        "source": "point"
                    })
                    print(f"[UDP] ID=590593573 点位数据: channel={channel_int}, X={x}, Y={y}, Z={z}")
                else:
                    print(f"[UDP] ID=590593573 未找到 -1 或 0 通道数据，忽略")
                return

            # 只处理指定ProcessorID的数据包
            if processor_id != 590593574:
                print(f"[UDP] 忽略ProcessorID={processor_id}的数据包")
                return

            # 记录原始JSON包到log（无条件记录监听到的原始数据）
            #log_wmps_raw(processor_id, data_str)

            # 只接收设备明确标记为有效、且XYZ为有限数的通道坐标。
            coord_data = {}
            for channel, channel_info in json_data.get("coord", {}).items():
                valid_value = channel_info.get(
                    "Valid",
                    channel_info.get("valid", True),
                )
                if isinstance(valid_value, str):
                    is_valid = valid_value.strip().lower() not in {
                        "0", "false", "invalid", "no",
                    }
                else:
                    is_valid = bool(valid_value)
                if not is_valid:
                    continue

                coordinate = np.asarray([
                    channel_info.get("X", np.nan),
                    channel_info.get("Y", np.nan),
                    channel_info.get("Z", np.nan),
                ], dtype=float)
                if not np.all(np.isfinite(coordinate)):
                    continue
                coord_data[int(channel)] = coordinate.tolist()

            # 解析原始数据（key已是发射器ID，不是RPM）
            raw_data = {}
            for channel, trans_data in json_data.get("raw", {}).items():
                channel_int = int(channel)
                raw_data[channel_int] = {}
                for trans_id, trans_info in trans_data.items():
                    raw_data[channel_int][int(trans_id)] = [
                        trans_info.get("t1", 0.0),
                        trans_info.get("t2", 0.0),
                        trans_info.get("T", 0.0),
                        trans_info.get("LocalCount", 0)
                    ]

            # UDP模式下 all_raw 与 raw_data 相同（key已是发射器ID）
            processed_data = {
                'coord': coord_data,
                'raw': raw_data,
                'all_raw': raw_data,
                'source': 'udp'
            }

            self.data_received.emit(processor_id, processed_data)
            print(f"UDP数据已处理，处理器ID: {processor_id}, 通道数: {len(coord_data)}")

        except Exception as e:
            print(f"处理UDP数据出错: {str(e)}")
            print_exc()
