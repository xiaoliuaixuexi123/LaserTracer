# -*- coding: utf-8 -*-
"""wMPS 接收器数据控件 —— 可嵌入主界面的 wMPS_widget 中。"""
import time
from collections import deque

import numpy as np
from numpy import array
from PySide6.QtCore import QObject, QThread, QTimer, Signal as pyqtSignal
from PySide6.QtNetwork import QTcpSocket
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from communication import UdpReceiver, rawResolveThread, wMPSProcessorConnection
from wmps_convert import convert_vertices_to_center
from wmps_distance_constraint import transmitter_origins_from_extpara


CENTER_WINDOW_SECONDS = 3.0
CENTER_JUMP_REJECT_MM = 30.0
CENTER_REBASE_SAMPLE_COUNT = 3
COORDINATE_PACKET_MAX_AGE_SECONDS = 1.0


class WMPSReceiverWidget(QWidget):
    """可嵌入大界面的 wMPS 接收器数据控件。"""

    connect_signal = pyqtSignal(str, str, int)
    disconnect_signal = pyqtSignal()
    load_innpara_signal = pyqtSignal(str)
    load_extpara_signal = pyqtSignal(str)
    clear_signal = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._row_by_receiver = {}
        self._init_ui()

    def _init_ui(self):
        self.setObjectName("WMPSReceiverWidget")
        main_layout = QVBoxLayout(self)

        # ── 网络连接设置 ──
        network_group = QGroupBox("环境网络连接")
        network_group.setObjectName("networkGroup")
        network_layout = QFormLayout(network_group)

        self.modeCombo = QComboBox()
        self.modeCombo.setObjectName("modeCombo")
        self.modeCombo.addItems(["TCP", "UDP"])
        self.ipInput = QLineEdit("192.168.1.201")
        self.ipInput.setObjectName("ipInput")
        self.portInput = QLineEdit("8234")
        self.portInput.setObjectName("portInput")
        network_layout.addRow("通信方式:", self.modeCombo)
        network_layout.addRow("监听地址:", self.ipInput)
        network_layout.addRow("监听端口:", self.portInput)

        button_layout = QHBoxLayout()
        self.connectButton = QPushButton("初始化网络")
        self.connectButton.setObjectName("connectButton")
        self.disconnectButton = QPushButton("断开连接")
        self.disconnectButton.setObjectName("disconnectButton")
        self.loadInnparaButton = QPushButton("加载内参")
        self.loadInnparaButton.setObjectName("loadInnparaButton")
        self.loadExtparaButton = QPushButton("加载外参")
        self.loadExtparaButton.setObjectName("loadExtparaButton")
        self.clearButton = QPushButton("清空数据")
        self.clearButton.setObjectName("clearButton")
        button_layout.addWidget(self.connectButton)
        button_layout.addWidget(self.disconnectButton)
        button_layout.addWidget(self.loadInnparaButton)
        button_layout.addWidget(self.loadExtparaButton)
        button_layout.addWidget(self.clearButton)
        button_layout.addStretch()
        network_layout.addRow(button_layout)

        # ── 状态标签行 ──
        status_layout = QHBoxLayout()
        self.networkStatusLabel = QLabel("网络: 未初始化")
        self.networkStatusLabel.setObjectName("networkStatusLabel")
        self.innStatusLabel = QLabel("内参: 未加载")
        self.innStatusLabel.setObjectName("innStatusLabel")
        self.extStatusLabel = QLabel("外参: 未加载")
        self.extStatusLabel.setObjectName("extStatusLabel")
        self.receiverCountLabel = QLabel("接收器: 0")
        self.receiverCountLabel.setObjectName("receiverCountLabel")
        status_layout.addWidget(self.networkStatusLabel)
        status_layout.addWidget(self.innStatusLabel)
        status_layout.addWidget(self.extStatusLabel)
        status_layout.addWidget(self.receiverCountLabel)
        status_layout.addStretch()

        # ── 接收器数据表格 ──
        table_group = QGroupBox("接收器数据")
        table_group.setObjectName("tableGroup")
        table_layout = QVBoxLayout(table_group)
        self.receiversTable = QTableWidget(0, 8)
        self.receiversTable.setObjectName("receiversTable")
        self.receiversTable.setHorizontalHeaderLabels(
            ["接收器ID", "处理器ID", "通道", "发射站信号", "状态", "位置X", "位置Y", "位置Z"]
        )
        self.receiversTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.receiversTable.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table_layout.addWidget(self.receiversTable)

        main_layout.addWidget(network_group)
        main_layout.addLayout(status_layout)
        main_layout.addWidget(table_group, 1)

        # ── 中心坐标计算 ──
        center_group = QGroupBox("中心坐标 (3秒滑动窗口)", self)
        center_group.setObjectName("centerGroup")
        center_layout = QGridLayout(center_group)

        self.centerProcessorIdLabel = QLabel("--")
        self.centerProcessorIdLabel.setObjectName("centerProcessorIdLabel")
        center_layout.addWidget(QLabel("处理器ID:"), 0, 0)
        center_layout.addWidget(self.centerProcessorIdLabel, 0, 1)

        self._vertex_avg_labels = {}
        for vi, vid in enumerate(("0", "1", "2")):
            x_lbl = QLabel("N/A")
            y_lbl = QLabel("N/A")
            z_lbl = QLabel("N/A")
            for lbl in (x_lbl, y_lbl, z_lbl):
                lbl.setStyleSheet("font-family: Consolas, monospace;")
            x_lbl.setObjectName(f"v{vid}AvgX")
            y_lbl.setObjectName(f"v{vid}AvgY")
            z_lbl.setObjectName(f"v{vid}AvgZ")
            self._vertex_avg_labels[vid] = (x_lbl, y_lbl, z_lbl)
            row = vi + 1
            center_layout.addWidget(QLabel(f"顶点{vid} 平均:"), row, 0)
            center_layout.addWidget(x_lbl, row, 1)
            center_layout.addWidget(y_lbl, row, 2)
            center_layout.addWidget(z_lbl, row, 3)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setFrameShadow(QFrame.Sunken)
        center_layout.addWidget(sep, 4, 0, 1, 4)

        bold_font = QLabel().font()
        bold_font.setBold(True)
        self.centerX = QLabel("N/A")
        self.centerY = QLabel("N/A")
        self.centerZ = QLabel("N/A")
        for lbl in (self.centerX, self.centerY, self.centerZ):
            lbl.setFont(bold_font)
            lbl.setStyleSheet("font-family: Consolas, monospace; font-weight: bold;")
        self.centerX.setObjectName("centerX")
        self.centerY.setObjectName("centerY")
        self.centerZ.setObjectName("centerZ")
        center_layout.addWidget(QLabel("中心坐标:"), 5, 0)
        center_layout.addWidget(self.centerX, 5, 1)
        center_layout.addWidget(self.centerY, 5, 2)
        center_layout.addWidget(self.centerZ, 5, 3)

        self.centerGroup = center_group
        self.centerGroup.setVisible(False)

        # ── 采集坐标显示 ──
        collected_group = QGroupBox("采集坐标")
        collected_group.setObjectName("collectedGroup")
        collected_layout = QGridLayout(collected_group)
        self.collectedX = QLabel("--")
        self.collectedY = QLabel("--")
        self.collectedZ = QLabel("--")
        for lbl in (self.collectedX, self.collectedY, self.collectedZ):
            lbl.setFont(bold_font)
            lbl.setStyleSheet("font-family: Consolas, monospace; font-weight: bold; color: #0078D4;")
        self.collectedX.setObjectName("collectedX")
        self.collectedY.setObjectName("collectedY")
        self.collectedZ.setObjectName("collectedZ")
        collected_layout.addWidget(QLabel("X:"), 0, 0)
        collected_layout.addWidget(self.collectedX, 0, 1)
        collected_layout.addWidget(QLabel("Y:"), 0, 2)
        collected_layout.addWidget(self.collectedY, 0, 3)
        collected_layout.addWidget(QLabel("Z:"), 0, 4)
        collected_layout.addWidget(self.collectedZ, 0, 5)
        main_layout.addWidget(collected_group)

        self.connectButton.clicked.connect(self._emit_connect)
        self.disconnectButton.clicked.connect(self.disconnect_signal.emit)
        self.loadInnparaButton.clicked.connect(self._select_innpara)
        self.loadExtparaButton.clicked.connect(self._select_extpara)
        self.clearButton.clicked.connect(self.clear_signal.emit)
        self.modeCombo.currentTextChanged.connect(self._sync_default_network)

    def _sync_default_network(self, mode):
        if mode == "UDP":
            self.ipInput.setText("127.0.0.1")
            self.portInput.setText("10000")
        else:
            self.ipInput.setText("192.168.1.201")
            self.portInput.setText("8234")

    def _emit_connect(self):
        try:
            port = int(self.portInput.text().strip())
            if not 1 <= port <= 65535:
                raise ValueError("端口范围应为 1-65535")
        except ValueError as exc:
            QMessageBox.warning(self, "端口错误", f"请输入有效端口号: {exc}")
            return
        ip = self.ipInput.text().strip()
        if not ip:
            QMessageBox.warning(self, "地址错误", "请输入监听地址")
            return
        self.connect_signal.emit(self.modeCombo.currentText(), ip, port)

    def _select_innpara(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "选择内参文件", "", "TXT文件 (*.txt);;所有文件 (*)")
        if file_path:
            self.load_innpara_signal.emit(file_path)

    def _select_extpara(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "选择外参文件", "", "TXT文件 (*.txt);;所有文件 (*)")
        if file_path:
            self.load_extpara_signal.emit(file_path)

    def set_network_status(self, text):
        self.networkStatusLabel.setText(f"网络: {text}")

    def set_param_status(self, inn_count=None, ext_count=None):
        if inn_count is not None:
            text = f"{inn_count} 组" if isinstance(inn_count, int) else str(inn_count)
            self.innStatusLabel.setText(f"内参: {text}")
        if ext_count is not None:
            text = f"{ext_count} 组" if isinstance(ext_count, int) else str(ext_count)
            self.extStatusLabel.setText(f"外参: {text}")

    def clear_table(self):
        self.receiversTable.setRowCount(0)
        self._row_by_receiver.clear()
        self.receiverCountLabel.setText("接收器: 0")

    def update_receivers(self, coordinates, statuses, transmitter_signals):
        for receiver_id in sorted(coordinates.keys(), key=self._receiver_sort_key):
            row = self._row_by_receiver.get(receiver_id)
            if row is None:
                row = self.receiversTable.rowCount()
                self.receiversTable.insertRow(row)
                self._row_by_receiver[receiver_id] = row

            processor_id, channel = receiver_id.split("@", 1)
            coords = coordinates[receiver_id]
            status_code = statuses.get(receiver_id, 2)
            status_text = "正常" if status_code == 1 else "异常"
            trans_ids = transmitter_signals.get(receiver_id, [])
            trans_text = ".".join(str(item) for item in trans_ids)

            values = [
                receiver_id,
                processor_id,
                channel,
                trans_text,
                status_text,
                f"{float(coords[0]):.3f}",
                f"{float(coords[1]):.3f}",
                f"{float(coords[2]):.3f}",
            ]
            for column, value in enumerate(values):
                self.receiversTable.setItem(row, column, QTableWidgetItem(value))

        self.receiverCountLabel.setText(f"接收器: {len(coordinates)}")

    @staticmethod
    def _receiver_sort_key(receiver_id):
        try:
            processor_id, channel = receiver_id.split("@", 1)
            return int(processor_id), int(channel)
        except ValueError:
            return receiver_id, 0

    # ── 中心坐标显示方法 ──

    def set_center_processor_id(self, pid):
        if pid is not None:
            self.centerProcessorIdLabel.setText(str(pid))
        else:
            self.centerProcessorIdLabel.setText("-- (等待数据)")

    def update_center_display(self, vertex_avg, center):
        for vid in ("0", "1", "2"):
            x_lbl, y_lbl, z_lbl = self._vertex_avg_labels[vid]
            avg = vertex_avg.get(vid)
            if avg is not None:
                x_lbl.setText(f"X={float(avg[0]):.4f}")
                y_lbl.setText(f"Y={float(avg[1]):.4f}")
                z_lbl.setText(f"Z={float(avg[2]):.4f}")
            else:
                x_lbl.setText("N/A")
                y_lbl.setText("N/A")
                z_lbl.setText("N/A")

        if center is not None:
            self.centerX.setText(f"X={float(center[0]):.4f}")
            self.centerY.setText(f"Y={float(center[1]):.4f}")
            self.centerZ.setText(f"Z={float(center[2]):.4f}")
        else:
            self.centerX.setText("N/A")
            self.centerY.setText("N/A")
            self.centerZ.setText("N/A")

    def clear_center_display(self):
        self.centerProcessorIdLabel.setText("--")
        for vid in ("0", "1", "2"):
            x_lbl, y_lbl, z_lbl = self._vertex_avg_labels[vid]
            x_lbl.setText("N/A")
            y_lbl.setText("N/A")
            z_lbl.setText("N/A")
        self.centerX.setText("N/A")
        self.centerY.setText("N/A")
        self.centerZ.setText("N/A")

    def set_collected_coords(self, x, y, z):
        """更新采集坐标显示"""
        self.collectedX.setText(f"{x:.4f}")
        self.collectedY.setText(f"{y:.4f}")
        self.collectedZ.setText(f"{z:.4f}")


class WMPSReceiverController(QObject):
    """绑定 WMPSReceiverWidget 的通信、参数加载和接收器数据解算控制器。"""

    transmitter_positions_changed = pyqtSignal(object)

    def __init__(self, widget, parent=None):
        super().__init__(parent)
        self.widget = widget
        self.m_processorRaw = {}
        self.m_processorCoordinate = {}
        self.m_receiverStatus = {}
        self.latest_coordinate_packets = {}
        self.use_udp = False
        self.udp_receiver = None
        self._cleaned_up = False

        self.m_wMPSThread = QThread(self)
        self.m_wMPSThread.start()
        self.m_wMPSProcessorConnection = wMPSProcessorConnection()
        self.m_wMPSProcessorConnection.moveToThread(self.m_wMPSThread)

        self.m_rawCalculate = rawResolveThread()
        self.m_wMPSProcessorConnection.rawArrived.connect(self.m_rawCalculate.calculate_Coord)
        self.m_rawCalculate.coordArrived.connect(self.solve_coordinate_packet)
        self.m_wMPSProcessorConnection.connection_status_changed.connect(self.handle_connection_status)

        self.widget.connect_signal.connect(self.initialize_network)
        self.widget.disconnect_signal.connect(self.disconnect_network)
        self.widget.load_innpara_signal.connect(self.load_innpara)
        self.widget.load_extpara_signal.connect(self.load_extpara)
        self.widget.clear_signal.connect(self.clear_data)
        self.widget.destroyed.connect(self.cleanup)

        # ── 最新中心坐标（供外部采集使用）──
        self.latest_center = None  # numpy array [X, Y, Z]

        # ── 中心坐标计算：3秒滑动窗口 ──
        self._vertex_buffers = {"0": deque(), "1": deque(), "2": deque()}
        self._last_good_vertex = {"0": None, "1": None, "2": None}
        self._pending_vertex_jump = {"0": None, "1": None, "2": None}
        self._center_processor_id = None
        self._center_timer = QTimer(self)
        self._center_timer.setInterval(100)
        self._center_timer.timeout.connect(self._recompute_center)
        self._center_timer.start()

    def initialize_network(self, mode, ip, port):
        try:
            self.disconnect_network(update_status=False)
            self._reset_center_state()
            self.use_udp = mode == "UDP"

            if self.use_udp:
                self.udp_receiver = UdpReceiver(ip=ip, port=port)
                self.udp_receiver.data_received.connect(self.handle_udp_data)
                if self.udp_receiver.start_listening():
                    self.widget.set_network_status(f"UDP已监听 {ip}:{port}")
                else:
                    self.widget.set_network_status(f"UDP监听失败 {ip}:{port}")
            else:
                success = self.m_wMPSProcessorConnection.reinitialize_server(ip, port)
                if success:
                    self.widget.set_network_status(f"TCP已监听 {ip}:{port}，等待处理器连接")
                else:
                    self.widget.set_network_status(f"TCP监听失败 {ip}:{port}")
        except Exception as exc:
            from traceback import print_exc
            print(f"初始化网络出错: {exc}")
            print_exc()
            self.widget.set_network_status(f"初始化失败: {exc}")

    def disconnect_network(self, update_status=True):
        try:
            if self.udp_receiver:
                self.udp_receiver.stop_listening()
                self.udp_receiver = None

            if self.m_wMPSProcessorConnection.m_tcpServer.isListening():
                self.m_wMPSProcessorConnection.m_tcpServer.close()

            for addr in list(self.m_wMPSProcessorConnection.m_processor_connection.keys()):
                sock = self.m_wMPSProcessorConnection.m_processor_connection[addr]
                if sock.state() == QTcpSocket.ConnectedState:
                    sock.close()
            self.m_wMPSProcessorConnection.m_processor_connection.clear()

            self.latest_coordinate_packets.clear()
            for receiver_id in self.m_receiverStatus:
                self.m_receiverStatus[receiver_id] = 2
            self._reset_center_state()
            self.update_display()

            if update_status:
                self.widget.set_network_status("已断开")
        except Exception as exc:
            from traceback import print_exc
            print(f"断开网络出错: {exc}")
            print_exc()
            if update_status:
                self.widget.set_network_status(f"断开失败: {exc}")

    def handle_connection_status(self, success, status_data):
        if success:
            self.widget.set_network_status(f"处理器已连接: {status_data.get('Timestamp', '')}")
        else:
            self.widget.set_network_status("未检测到处理器连接")

    def load_innpara(self, file_path):
        if self.m_rawCalculate.read_innpara_from_txt(file_path):
            self.widget.set_param_status(inn_count=len(self.m_rawCalculate.m_innPara))
        else:
            self.widget.set_param_status(inn_count="加载失败")

    def load_extpara(self, file_path):
        if self.m_rawCalculate.read_extpara_from_txt(file_path):
            self.widget.set_param_status(ext_count=len(self.m_rawCalculate.m_extPara))
            self.transmitter_positions_changed.emit({
                "positions": self.get_transmitter_origins_global_mm(),
                "source": str(file_path),
            })
        else:
            self.widget.set_param_status(ext_count="加载失败")

    def get_transmitter_origins_global_mm(self):
        """Return station centers using the active real-time solver extrinsics."""
        return transmitter_origins_from_extpara(self.m_rawCalculate.m_extPara)

    def get_latest_four_marker_snapshot(
        self,
        max_age_seconds=COORDINATE_PACKET_MAX_AGE_SECONDS,
    ):
        """Return one coherent, recent four-channel coordinate packet.

        Channel order 0/1/2/3 (or explicitly 1/2/3/4) maps to calibrated
        marker order 1/2/3/4.  Coordinates from different processors or old
        cached packets are never combined.
        """
        now = time.monotonic()
        max_age = float(max_age_seconds)
        for processor_id in sorted(self.latest_coordinate_packets):
            packet = self.latest_coordinate_packets[processor_id]
            age_seconds = now - float(packet["timestamp_monotonic"])
            if age_seconds < 0.0 or age_seconds > max_age:
                continue
            coordinates = packet["coordinates"]
            for channel_order in ((0, 1, 2, 3), (1, 2, 3, 4)):
                if not all(channel in coordinates for channel in channel_order):
                    continue
                marker_coordinates = np.vstack(
                    [coordinates[channel] for channel in channel_order]
                )
                if marker_coordinates.shape != (4, 3):
                    continue
                if not np.all(np.isfinite(marker_coordinates)):
                    continue
                raw_channels = self.m_processorRaw.get(processor_id, {})
                station_sets = [
                    set(raw_channels.get(channel, {}).keys())
                    for channel in channel_order
                    if raw_channels.get(channel)
                ]
                if station_sets:
                    common_stations = set.intersection(*station_sets)
                    station_ids = common_stations or set.union(*station_sets)
                else:
                    station_ids = set()
                return {
                    "processor_id": int(processor_id),
                    "channel_order": tuple(channel_order),
                    "receiver_ids": tuple(
                        f"{processor_id}@{channel}" for channel in channel_order
                    ),
                    "coordinates": marker_coordinates.copy(),
                    "age_seconds": float(age_seconds),
                    "source": packet.get("source", "unknown"),
                    "transmitter_ids": tuple(sorted(int(v) for v in station_ids)),
                }
        return None

    def handle_udp_data(self, processor_id, data):
        if processor_id == 590593573 and data.get("source") == "point":
            self.register_point_as_channel(processor_id, data.get("channel", 0), data["point"])
            return
        self.solve_coordinate_packet(processor_id, data)

    def register_point_as_channel(self, processor_id, channel, point):
        try:
            receiver_id = f"{processor_id}@{channel}"
            self.m_processorRaw.setdefault(processor_id, {}).setdefault(channel, {})[-1] = [0.0, 0.0]
            coordinate = array(
                [float(point.get("X", 0.0)), float(point.get("Y", 0.0)), float(point.get("Z", 0.0))]
            )
            self.m_processorCoordinate[receiver_id] = coordinate
            self.m_receiverStatus[receiver_id] = 1

            # A point packet is an atomic one-channel sample; never merge it
            # with earlier packets to manufacture a four-marker snapshot.
            self.latest_coordinate_packets[int(processor_id)] = {
                "timestamp_monotonic": time.monotonic(),
                "coordinates": {int(channel): coordinate.copy()},
                "source": "point",
            }
            self.update_display()
        except Exception as exc:
            print(f"注册点位数据出错: {exc}")

    def solve_coordinate_packet(self, processor_id, coord_dict):
        try:
            self.m_processorRaw[processor_id] = {}
            all_raw = coord_dict.get("all_raw", {})
            is_udp = coord_dict.get("source") == "udp"

            for channel_key, trans_dict in all_raw.items():
                channel = int(channel_key)
                self.m_processorRaw[processor_id][channel] = {}
                for trans_key, values in trans_dict.items():
                    trans_id = int(trans_key)
                    if is_udp:
                        display_trans_id = trans_id
                    else:
                        display_trans_id = self.m_rawCalculate.RPM_2_trans.get(trans_id, trans_id)
                    self.m_processorRaw[processor_id][channel][display_trans_id] = [values[0], values[1]]

            raw_channels = coord_dict.get("raw", {})
            coord_channels = coord_dict.get("coord", {})
            packet_coordinates = {}
            for channel_key, coords in coord_channels.items():
                coordinate = np.asarray(coords, dtype=float).reshape(-1)
                if coordinate.size >= 3 and np.all(np.isfinite(coordinate[:3])):
                    packet_coordinates[int(channel_key)] = coordinate[:3].copy()

            self.latest_coordinate_packets[int(processor_id)] = {
                "timestamp_monotonic": time.monotonic(),
                "coordinates": packet_coordinates,
                "source": "udp" if is_udp else "tcp",
            }

            channel_keys = set(raw_channels.keys()) | set(coord_channels.keys())
            for channel_key in channel_keys:
                channel = int(channel_key)
                receiver_id = f"{processor_id}@{channel}"

                coord_key = channel if channel in coord_channels else str(channel)
                if coord_key in coord_channels:
                    coords = coord_channels[coord_key]
                    self.m_processorCoordinate[receiver_id] = array(
                        [float(coords[0]), float(coords[1]), float(coords[2])]
                    )
                    self.m_receiverStatus[receiver_id] = 1
                else:
                    self.m_processorCoordinate[receiver_id] = array([0.0, 0.0, 0.0])
                    self.m_receiverStatus[receiver_id] = 2

            current_channels = {int(channel) for channel in coord_channels}
            processor_prefix = f"{processor_id}@"
            for receiver_id in list(self.m_receiverStatus):
                if not receiver_id.startswith(processor_prefix):
                    continue
                try:
                    cached_channel = int(receiver_id.split("@", 1)[1])
                except (IndexError, ValueError):
                    continue
                if cached_channel not in current_channels:
                    self.m_receiverStatus[receiver_id] = 2

            self.update_display()
        except Exception as exc:
            from traceback import print_exc
            print(f"处理接收器数据包出错: {exc}")
            print_exc()

    def update_display(self):
        transmitter_signals = {}
        for processor_id, channels in self.m_processorRaw.items():
            for channel, trans_dict in channels.items():
                transmitter_signals[f"{processor_id}@{channel}"] = list(trans_dict.keys())
        self.widget.update_receivers(self.m_processorCoordinate, self.m_receiverStatus, transmitter_signals)

    # ── 中心坐标计算逻辑 ──

    def _detect_center_processor(self):
        """找到第一个同时拥有通道 0/1/2 且状态正常的处理器 ID。"""
        from collections import defaultdict
        processor_channels = defaultdict(set)
        for receiver_id, coords in self.m_processorCoordinate.items():
            status = self.m_receiverStatus.get(receiver_id, 2)
            if status != 1:
                continue
            try:
                pid_str, ch_str = receiver_id.split("@", 1)
                pid = int(pid_str)
                ch = int(ch_str)
            except ValueError:
                continue
            if ch in (0, 1, 2):
                processor_channels[pid].add(str(ch))
        for pid in sorted(processor_channels):
            if {"0", "1", "2"}.issubset(processor_channels[pid]):
                return pid
        return None

    def _recompute_center(self):
        """Timer 回调：每 100ms 更新滑动窗口内的均值与中心坐标。"""
        now = time.time()

        if self._center_processor_id is None:
            pid = self._detect_center_processor()
            if pid is not None:
                self._center_processor_id = pid
                self.widget.set_center_processor_id(pid)
            else:
                return

        pid = self._center_processor_id
        for ch in ("0", "1", "2"):
            key = f"{pid}@{ch}"
            if key in self.m_processorCoordinate:
                latest = self.m_processorCoordinate[key]
                self._append_center_sample(ch, latest, now)

        for ch in ("0", "1", "2"):
            buf = self._vertex_buffers[ch]
            while buf and now - buf[0][0] > CENTER_WINDOW_SECONDS:
                buf.popleft()

        vertex_avg = {}
        for ch in ("0", "1", "2"):
            vertex_avg[ch] = self._robust_average(self._vertex_buffers[ch])

        if all(vertex_avg[ch] is not None for ch in ("0", "1", "2")):
            cx, cy, cz = convert_vertices_to_center(
                vertex_avg["0"], vertex_avg["1"], vertex_avg["2"]
            )
            center = np.array([cx, cy, cz])
            self.latest_center = center.copy()
        else:
            center = None
            self.latest_center = None

        self.widget.update_center_display(vertex_avg, center)

    def _append_center_sample(self, ch, latest, now):
        latest = np.asarray(latest, dtype=float)
        if latest.shape[0] < 3 or not np.all(np.isfinite(latest[:3])):
            return
        latest = latest[:3]

        buf = self._vertex_buffers[ch]
        if buf and np.array_equal(buf[-1][1], latest):
            return

        last_good = self._last_good_vertex[ch]
        if last_good is None:
            self._accept_center_sample(ch, latest, now)
            return

        jump_mm = float(np.linalg.norm(latest - last_good))
        if jump_mm <= CENTER_JUMP_REJECT_MM:
            self._pending_vertex_jump[ch] = None
            self._accept_center_sample(ch, latest, now)
            return

        pending = self._pending_vertex_jump[ch]
        if pending is None:
            self._pending_vertex_jump[ch] = (latest.copy(), 1)
            return

        pending_value, pending_count = pending
        if float(np.linalg.norm(latest - pending_value)) <= CENTER_JUMP_REJECT_MM:
            pending_count += 1
        else:
            pending_value = latest.copy()
            pending_count = 1

        if pending_count >= CENTER_REBASE_SAMPLE_COUNT:
            buf.clear()
            self._pending_vertex_jump[ch] = None
            self._accept_center_sample(ch, latest, now)
        else:
            self._pending_vertex_jump[ch] = (pending_value, pending_count)

    def _accept_center_sample(self, ch, latest, now):
        sample = latest.copy()
        self._vertex_buffers[ch].append((now, sample))
        self._last_good_vertex[ch] = sample

    def _robust_average(self, buf):
        if not buf:
            return None

        values = np.array([entry[1] for entry in buf], dtype=float)
        if values.size == 0:
            return None

        if len(values) < 3:
            return np.mean(values, axis=0)

        median = np.median(values, axis=0)
        dist = np.linalg.norm(values - median, axis=1)
        keep = dist <= CENTER_JUMP_REJECT_MM
        if not np.any(keep):
            return median
        return np.mean(values[keep], axis=0)

    def _reset_center_state(self):
        for ch in ("0", "1", "2"):
            self._vertex_buffers[ch].clear()
            self._last_good_vertex[ch] = None
            self._pending_vertex_jump[ch] = None
        self._center_processor_id = None
        self.latest_center = None
        self.widget.clear_center_display()

    def clear_data(self):
        self.m_processorRaw.clear()
        self.m_processorCoordinate.clear()
        self.m_receiverStatus.clear()
        self.latest_coordinate_packets.clear()
        self.widget.clear_table()
        self._reset_center_state()

    def cleanup(self):
        if self._cleaned_up:
            return
        self._cleaned_up = True
        self._center_timer.stop()
        self.disconnect_network(update_status=False)
        if self.m_wMPSThread.isRunning():
            self.m_wMPSThread.quit()
            self.m_wMPSThread.wait(3000)


def create_wmps_receiver_widget(parent=None):
    """工厂函数：返回 (widget, controller)，主界面需要保存 controller 引用。"""
    widget = WMPSReceiverWidget(parent)
    controller = WMPSReceiverController(widget, parent=widget)
    return widget, controller
