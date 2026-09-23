import cv2
from PySide6.QtCore import (
    QObject, Signal, QThread, Qt, QByteArray, QTimer, Slot
)
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QApplication
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtNetwork import QTcpSocket, QAbstractSocket, QUdpSocket, QHostAddress
import json
import re
import numpy as np
from collections import deque
import time
import pickle
import sys

class LaserDataProcessor(QObject):
    """
    单通道激光测距数据处理器（最终确认版）

    数据特性（按你的真实设备）：
    - ng：只在测量开始时出现一次 → 测量级慢变量
    - fr1：每一组都会出现 → 组边界触发
    - 第一通道：目标相关 → 用于测距
    - 第二通道：稳定参考 → 只做统计输出

    处理原则：
    - fr1 到来 → 结算上一组
    - 每一组独立输出
    - 不做“组复位”
    - 只在 finalize_results() 时统一复位
    """

    raw_data_received = Signal(str)
    processed_result = Signal(dict)   # 每一组结果
    final_result = Signal(dict)       # 所有组平均
    error_detected = Signal(str)

    # ------------------------------------------------------------
    def __init__(
        self,
        rude_distance=0,
        buffer_size=1000,
        ref_min=1571.0,
        ref_max=1573.0
    ):
        super().__init__()

        self.rude_distance = rude_distance
        self.data_buffer = deque(maxlen=buffer_size)

        # 第二通道（稳定参考）识别范围
        self.ref_min = float(ref_min)
        self.ref_max = float(ref_max)
        self.avg_2 = 1572.0

        # ===== 测量级状态 =====
        self.current_ng = None      # 慢变量，只要来一次即可

        # ===== 组级状态 =====
        self.current_fr1 = None     # 每组更新
        self.raw_ch1 = []           # 第一通道（目标）
        self.raw_ch2 = []           # 第二通道（参考）

        # ===== 统计 =====
        self.group_index = 0
        self.all_group_distances = []

    # ------------------------------------------------------------
    @Slot(float)
    def set_rude_distance(self, distance):
        self.rude_distance = distance

    # ------------------------------------------------------------
    @Slot(str)
    def process_line(self, line):
        if not line:
            return

        line = line.strip()
        self.data_buffer.append(line)
        self.raw_data_received.emit(line)

        # ---- 错误行 ----
        if self._is_error_data(line):
            self.error_detected.emit(line)
            return

        # ---- ng（慢变量，只需一次）----
        m_ng = re.search(r'ng\s*=\s*([\d.]+)', line)
        if m_ng:
            self.current_ng = float(m_ng.group(1))
            return

        # ---- fr1：结算上一组 ----
        m_fr1 = re.search(r'fr1\s*=\s*([\d.]+)\s*MHz', line)
        if m_fr1:
            self._finalize_group()
            self.current_fr1 = float(m_fr1.group(1))
            return

        # ---- 数据行：数值,方差 ----
        m_val = re.match(
            r'^\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,',
            line
        )
        if not m_val:
            return

        try:
            value = float(m_val.group(1))
        except ValueError:
            return

        # ---- 通道区分 ----
        if self.ref_min <= value <= self.ref_max:
            self.raw_ch2.append(value)   # 稳定参考
        else:
            self.raw_ch1.append(value)   # 目标相关

    # ------------------------------------------------------------
    def _finalize_group(self):
        """
        结算一组：
        - 必须有第一通道数据
        - ng 只要在本次测量中出现过即可
        """

        if not self.raw_ch1:
            return
        if self.current_ng is None:
            return
        if self.current_fr1 is None:
            return

        self.group_index += 1

        # 第一通道平均
        avg_ch1 = float(np.mean(self.raw_ch1))

        # 第二通道平均（仅输出，不参与测距）
        avg_ch2 = float(np.mean(self.raw_ch2)) if self.raw_ch2 else None

        # 粗测展开
        c = 3e8
        lpp = c / (2 * self.current_fr1 * 1e6 * self.current_ng) * 1e3
        n = int((self.rude_distance + 4400.0) // lpp)
        distance = avg_ch1 + n * lpp - 4760.29388 + 1572.100 - self.avg_2

        # 保存用于“所有组平均”
        self.all_group_distances.append(distance)

        # 输出本组结果
        self.processed_result.emit({
            "group_index": self.group_index,
            "avg_ch1_mm": avg_ch1,
            "avg_ch2_mm": avg_ch2,
            "distance_mm": distance,
            "samples_ch1": len(self.raw_ch1),
            "samples_ch2": len(self.raw_ch2),
            "fr1": self.current_fr1,
            "ng": self.current_ng,
            "lpp_mm": lpp,
            "n": n,
            "timestamp": time.time()
        })

        print(
    f"DEBUG: avg={avg_ch1:.3f}, n={n}, "
    f"n*lpp={n*lpp:.3f}, distance={distance:.3f}"
)


        # ⚠️ 只清 raw，绝不清 ng / fr1
        self.raw_ch1.clear()
        self.raw_ch2.clear()

    # ------------------------------------------------------------
    @Slot()
    def finalize_results(self):
        """
        测量结束时调用：
        - 输出所有组平均
        - 然后统一复位
        """

        # 防止最后一组没被 fr1 触发
        self._finalize_group()

        if not self.all_group_distances:
            return

        final_mean = float(np.mean(self.all_group_distances))
        final_std = (
            float(np.std(self.all_group_distances, ddof=1))
            if len(self.all_group_distances) > 1 else 0.0
        )

        self.final_result.emit({
            "groups_used": len(self.all_group_distances),
            "final_mean_mm": final_mean,
            "final_std_mm": final_std
        })

        # ===== 整次测量结束，统一复位 =====
        self.all_group_distances.clear()
        self.group_index = 0
        self.current_ng = None
        self.current_fr1 = None
        self.raw_ch1.clear()
        self.raw_ch2.clear()

    # ------------------------------------------------------------
    @staticmethod
    def _is_error_data(line):
        patterns = [
            r'NO DATA',
            r'Data error',
            r'DMA data error',
            r'ERROR',
            r'TIMEOUT',
            r'FAIL'
        ]
        return any(re.search(p, line, re.IGNORECASE) for p in patterns)




# class LaserDataProcessor(QObject):
#     """单通道激光测距数据处理器
#     fr1 到来即结算上一组，最终再算所有组的平均值
#     """

#     raw_data_received = Signal(str)
#     processed_result = Signal(dict)   # 单组结果
#     final_result = Signal(dict)       # 全部组的最终平均
#     error_detected = Signal(str)

#     def __init__(self, rude_distance=0, buffer_size=1000):
#         super().__init__()

#         self.rude_distance = rude_distance
#         self.data_buffer = deque(maxlen=buffer_size)

#         # ===== 物理状态（上一组）=====
#         self.current_ng = None
#         self.current_fr1 = None

#         # ===== 当前组原始数据 =====
#         self.raw_values = []

#         # ===== 历史展开后的组结果 =====
#         self.group_distances = []

#     # ------------------------------------------------------------------
#     @Slot(float)
#     def set_rude_distance(self, distance):
#         self.rude_distance = distance

#     @Slot()
#     def reset(self):
#         self.current_ng = None
#         self.current_fr1 = None
#         self.raw_values.clear()
#         self.group_distances.clear()

#     # ------------------------------------------------------------------
#     @Slot(str)
#     def process_line(self, line):
#         if not line:
#             return

#         line = line.strip()
#         self.data_buffer.append(line)
#         self.raw_data_received.emit(line)

#         # ---- 错误行 ----
#         if self._is_error_data(line):
#             self.error_detected.emit(line)
#             return

#         # ---- ng ----
#         m_ng = re.search(r'ng\s*=\s*([\d.]+)', line)
#         if m_ng:
#             self.current_ng = float(m_ng.group(1))
#             return

#         # ---- fr1：先结算上一组，再更新 ----
#         m_fr1 = re.search(r'fr1\s*=\s*([\d.]+)\s*MHz', line)
#         if m_fr1:
#             self._finalize_previous_group()
#             self.current_fr1 = float(m_fr1.group(1))
#             return

#         # ---- 数据行：距离,方差 ----
#         m_val = re.match(
#             r'^\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,',
#             line
#         )
#         if not m_val:
#             return

#         try:
#             value = float(m_val.group(1))
#         except ValueError:
#             return

#         self.raw_values.append(value)
#         value = None

#     # ------------------------------------------------------------------
#     def _finalize_previous_group(self):
#         """用【旧 fr1】结算上一组"""

#         if not self.raw_values:
#             return
#         if self.current_fr1 is None or self.current_ng is None:
#             self.raw_values.clear()
#             return

#         raw_avg = float(np.mean(self.raw_values))

#         # ---- 展开公式 ----
#         c = 3e8
#         lpp = c / (2 * self.current_fr1 * 1e6 * self.current_ng) * 1e3
#         n = int(self.rude_distance // lpp)
#         distance = raw_avg + n * lpp

#         self.group_distances.append(distance)

#         # ---- 实时输出：单组结果 ----
#         self.processed_result.emit({
#             "group_index": len(self.group_distances),
#             "samples": len(self.raw_values),
#             "ng": self.current_ng,
#             "fr1": self.current_fr1,
#             "lpp_mm": lpp,
#             "n": n,
#             "distance_mm": distance,
#             "timestamp": time.time()
#         })

#         self.raw_values.clear()

#     # ------------------------------------------------------------------
#     def finalize_results(self):
#         """测量结束时调用：算所有组的最终平均"""

#         # 把最后一组也结算掉
#         self._finalize_previous_group()

#         if not self.group_distances:
#             return

#         final_mean = float(np.mean(self.group_distances))
#         final_std  = float(np.std(self.group_distances, ddof=1)) \
#                      if len(self.group_distances) > 1 else 0.0
        
#         self.group_distances= []
#         self.current_ng = None
#         self.current_fr1 = None
#         self.raw_values = []

#         self.final_result.emit({
#             "final_mean_mm": final_mean,
#             "final_std_mm": final_std,
#             "groups_used": len(self.group_distances)
#         })

#     # ------------------------------------------------------------------
#     @staticmethod
#     def _is_error_data(line):
#         patterns = [
#             r'NO DATA',
#             r'Data error',
#             r'DMA data error',
#             r'ERROR',
#             r'TIMEOUT',
#             r'FAIL'
#         ]
#         return any(re.search(p, line, re.IGNORECASE) for p in patterns)


# class LaserDataProcessor(QObject):
#     """测距仪数据处理器 - 独立运行在线程中"""

#     raw_data_received = Signal(str)     # 原始数据行
#     processed_result = Signal(dict)     # 实时处理结果
#     final_result = Signal(dict)         # 停止测量时的最终结果
#     error_detected = Signal(str)        # 错误数据

#     def __init__(self, rude_distance=0, buffer_size=1000):
#         super().__init__()
#         self.rude_distance = rude_distance
#         self.data_buffer = deque(maxlen=buffer_size)

#         self.current_ng = None
#         self.current_fr1 = None
#         self.channel1_values = []
#         self.channel2_values = []
#         self.data_line_position = 0

#         self.channel1_avg_values = []
#         self.channel2_avg_values = []

#     @Slot(float)
#     def set_rude_distance(self, distance):
#         """设置粗测值（在线程内执行）"""
#         self.rude_distance = distance

#     @Slot()
#     def reset(self):
#         """重置处理器状态"""
#         self.current_ng = 1.000269
#         self.current_fr1 = None
#         self.channel1_values = []
#         self.channel2_values = []
#         self.data_line_position = 0
#         self.channel1_avg_values = []
#         self.channel2_avg_values = []
#         # 可以保留少量 print，但建议注释
#         # print("数据处理器已重置")

#     def get_buffer_data(self):
#         """获取缓冲区所有数据（注意：跨线程调用时自行小心）"""
#         return list(self.data_buffer)
    
#     @Slot(str)
#     def process_line(self, line):
#         """实时处理每一行数据（在工作线程中跑）"""
#         if not line:
#             return

#         line = line.strip()
#         self.data_buffer.append(line)
#         self.raw_data_received.emit(line)

#         # 1) ng 行
#         ng_match = re.search(r'ng\s*=\s*([\d.]+)', line)
#         if ng_match:
#             self.current_ng = float(ng_match.group(1))
#             return

#         # 2) fr1 行 —— 表示上一组数据结束，可计算
#         fr1_match = re.search(r'fr1\s*=\s*([\d.]+)\s*MHz', line)
#         if fr1_match:
#             # 先更新当前 fr1（这次测量用的就是这个频率）
#             self.current_fr1 = float(fr1_match.group(1))

#             # 上一组有数据 → 计算平均
#             if self.channel1_values and self.channel2_values:
#                 self._calculate_group_average()

#             # 清空准备下一组
#             self.channel1_values = []
#             self.channel2_values = []
#             self.data_line_position = 0
#             return

#         # 3) 数据行：格式类似  1331.278,4.534  （前面是距离，后面是方差）
#         #    我们只用逗号前面的“距离”作为有效值
#         if not self._is_error_data(line):
#             # 尝试匹配 “距离,方差” 这一类行
#             m = re.match(r'^\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,', line)
#             if m:
#                 try:
#                     value = float(m.group(1))   # 只取逗号前面的距离
#                 except ValueError:
#                     value = None

#                 if value is not None:
#                     # 奇偶行分配到 ch1 / ch2
#                     if self.data_line_position % 2 == 0:
#                         self.channel1_values.append(value)
#                     else:
#                         self.channel2_values.append(value)

#                     self.data_line_position += 1
#                     return

#             # 如果不是“距离,方差”，再退回到你原来的通用解析（防止将来有其它格式）
#             value, is_valid = self._extract_numeric_value(line)
#             if is_valid and value is not None:
#                 if self.data_line_position % 2 == 0:
#                     self.channel1_values.append(value)
#                 else:
#                     self.channel2_values.append(value)

#                 self.data_line_position += 1
#                 return

#         # 4) 错误行（包括 data error）
#         if self._is_error_data(line):
#             self.error_detected.emit(line)
#             return



#     def _calculate_group_average(self):
#         """计算当前频率组平均值并发送结果"""

#         # print(f"fr1:{self.current_fr1}, ng:{self.current_ng}")

#         if not self.current_ng or not self.current_fr1:
#             return

#         c = 3e8
#         lpp = c / (2 * self.current_fr1 * 1e6 * self.current_ng) * 1e3  # mm
#         # print(f"lpp: {lpp} mm")

#         n = self.rude_distance // lpp
#         print(self.rude_distance, lpp, n)
#         # print(f"n: {n}")

#         # 这里一组就是你说的“4 个数”（4 行），每行 ch1 一个值、ch2 一个值
#         avg_ch1 = np.mean(self.channel1_values) 
#         avg_ch2 = np.mean(self.channel2_values) + n * lpp
#         # print(f"avg_ch1: {avg_ch1} mm, avg_ch2: {avg_ch2} mm")

#         self.channel1_avg_values.append(avg_ch1)
#         self.channel2_avg_values.append(avg_ch2)

#         total_avg_ch1 = np.mean(self.channel1_avg_values)
#         total_avg_ch2 = np.mean(self.channel2_avg_values)

#         result = {
#             'ng': self.current_ng,
#             'fr1': self.current_fr1,
#             'channel1_current': avg_ch1,
#             'channel2_current': avg_ch2,
#             'distance1': total_avg_ch1,
#             'distance2': total_avg_ch2,
#             'groups_processed': len(self.channel1_avg_values),
#             'ch1_samples': len(self.channel1_values),
#             'ch2_samples': len(self.channel2_values),
#             'timestamp': time.time()
#         }

#         self.processed_result.emit(result)

        
#     @staticmethod
#     def _is_error_data(line):
#         error_patterns = [
#             r'NO DATA!',
#             r'Data error',
#             r'DMA data error',
#             r'ERROR',
#             r'TIMEOUT',
#             r'FAIL'
#         ]
#         for pattern in error_patterns:
#             if re.search(pattern, line, re.IGNORECASE):
#                 return True
#         return False

#     @staticmethod
#     def _extract_numeric_value(line):
#         if LaserDataProcessor._is_error_data(line):
#             return None, False

#         match = re.match(r'^\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', line)
#         if match:
#             try:
#                 value = float(match.group(1))
#                 return value, True
#             except ValueError:
#                 return None, False
#         return None, False

#     def finalize_results(self):
#         """
#         在停止测量时调用：将所有组进行全局平均 + 滑动平均 + 指数滤波
#         """
#         if not self.channel1_avg_values or not self.channel2_avg_values:
#             return None

#         # 1）全局最简单平均
#         global_ch1 = np.mean(self.channel1_avg_values)
#         global_ch2 = np.mean(self.channel2_avg_values)

#         # 2）滑动平均（窗口可调）
#         def moving_average(data, window=5):
#             if len(data) < window:
#                 return np.mean(data)
#             return np.mean(data[-window:])

#         smooth_ch1_ma = moving_average(self.channel1_avg_values, window=5)
#         smooth_ch2_ma = moving_average(self.channel2_avg_values, window=5)

#         # 3）指数滤波（α 可调）
#         def exp_smooth(data, alpha=0.3):
#             smoothed = data[0]
#             for d in data[1:]:
#                 smoothed = alpha * d + (1 - alpha) * smoothed
#             return smoothed

#         smooth_ch1_exp = exp_smooth(self.channel1_avg_values, alpha=0.3)
#         smooth_ch2_exp = exp_smooth(self.channel2_avg_values, alpha=0.3)

#         # μm 级输出
#         result =  {
#             "global_ch1_mm": global_ch1,
#             "global_ch2_mm": global_ch2,
#             "smooth_ma_ch1_mm": smooth_ch1_ma,
#             "smooth_ma_ch2_mm": smooth_ch2_ma,
#             "smooth_exp_ch1_mm": smooth_ch1_exp,
#             "smooth_exp_ch2_mm": smooth_ch2_exp,
#             "final_ch1_um": smooth_ch1_exp,
#             "final_ch2_um": smooth_ch2_exp,
#         }
#         self.final_result.emit(result)


# ====================== 网络工作线程：解析 JSON + 激光处理 ======================

class NetworkWorker(QObject):
    """运行在独立 QThread 中：解析 JSON + 调用激光处理器"""

    motor_response = Signal(dict)
    motor_feedback = Signal(dict)
    laser_raw_data = Signal(str)
    laser_processed_data = Signal(dict)
    laser_final_data = Signal(dict)
    laser_error = Signal(str)

    def __init__(self, rude_distance=0, parent=None):
        super().__init__(parent)
        self.laser_processor = LaserDataProcessor(rude_distance=rude_distance)

        # 把内部激光信号转发出来
        self.laser_processor.raw_data_received.connect(self.laser_raw_data)
        self.laser_processor.processed_result.connect(self.laser_processed_data)
        self.laser_processor.final_result.connect(self.laser_final_data)
        self.laser_processor.error_detected.connect(self.laser_error)

    @Slot(str)
    def process_json_line(self, line: str):
        """主线程把每一行 JSON 文本丢进来，这里解析"""
        line = line.strip()
        if not line:
            return

        try:
            response = json.loads(line)
        except json.JSONDecodeError:
            # JSON 解析错误直接忽略，避免阻塞
            return

        # 判断类型
        if response.get('device') == 'laser' and response.get('type') == 'data_stream':
            laser_data = response.get('data', '')
            if laser_data:
                self.laser_processor.process_line(laser_data)
        elif response.get('device') == 'motor' and response.get('type') == 'data_stream':
            # Periodic motor feedback is kept separate from command responses,
            # preventing the main window from logging every streamed sample.
            self.motor_feedback.emit(response)
        else:
            # 其他都当作“控制响应”丢给上层
            self.motor_response.emit(response)

    # 下面几个是主线程控制激光处理器用的

    @Slot(float)
    def set_rude_distance(self, distance: float):
        self.laser_processor.set_rude_distance(distance)

    @Slot()
    def reset_laser_processor(self):
        self.laser_processor.reset()

    @Slot()
    def finalize_laser(self):
        self.laser_processor.finalize_results()



# ====================== 上位机控制器（主线程：只负责收发 / 派发） ======================

class RobotNetworkController(QObject):
    """上位机: 电机控制 + 测距仪数据，完全异步结构"""

    connected = Signal()
    disconnected = Signal()
    error_occurred = Signal(str)
    motor_response_received = Signal(dict)
    motor_feedback_received = Signal(dict)
    motor_temperature_received = Signal(str, float)

    # 对外暴露的激光信号
    laser_raw_data = Signal(str)
    laser_processed_data = Signal(dict)
    laser_final_data = Signal(dict)
    finalize_laser_signal = Signal()
    laser_error = Signal(str)

    # 发送给工作线程的信号
    json_line_received = Signal(str)
    set_rude_distance_signal = Signal(float)
    reset_laser_signal = Signal()

    def __init__(self, host='192.168.1.100', port=8888, parent=None):
        super().__init__(parent)
        self.rude_distance = 0

        self.host = host
        self.port = port

        # TCP Socket 在主线程, 只负责 IO
        self.socket = QTcpSocket(self)
        self.socket.connected.connect(self._on_connected)
        self.socket.disconnected.connect(self._on_disconnected)
        self.socket.errorOccurred.connect(self._on_error)
        self.socket.readyRead.connect(self._on_data_received)
        self.motors = {}
        self.last_motor_feedback = {}
        self.last_motor_temperatures = {}
        self.buffer = ""

        # ===== 网络工作线程 =====
        self.network_thread = QThread(self)
        self.worker = NetworkWorker(rude_distance=self.rude_distance)
        self.worker.moveToThread(self.network_thread)
        

        # 主线程 -> 工作线程
        self.json_line_received.connect(
            self.worker.process_json_line,
            Qt.QueuedConnection
        )
        self.set_rude_distance_signal.connect(
            self.worker.set_rude_distance,
            Qt.QueuedConnection
        )
        self.reset_laser_signal.connect(
            self.worker.reset_laser_processor,
            Qt.QueuedConnection
        )

        # 工作线程 -> 主线程
        self.worker.motor_response.connect(self._cache_motor_response)
        self.worker.motor_response.connect(self.motor_response_received)
        self.worker.motor_feedback.connect(self._cache_motor_response)
        self.worker.motor_feedback.connect(self.motor_feedback_received)
        self.worker.laser_raw_data.connect(self.laser_raw_data)
        self.worker.laser_processed_data.connect(self.laser_processed_data)
        self.worker.laser_final_data.connect(self.laser_final_data)
        self.worker.laser_error.connect(self.laser_error)

        self.network_thread.start()
        self.finalize_laser_signal.connect(self.worker.finalize_laser)
    # ========== 连接相关 ==========

    def connectToRaspberryPi(self):
        if self.socket.state() == QAbstractSocket.UnconnectedState:
            self.socket.connectToHost(self.host, self.port)

    def disconnect(self):
        if self.socket.state() == QAbstractSocket.ConnectedState:
            self.socket.disconnectFromHost()

    def _on_connected(self):
        self.connected.emit()

    def _on_disconnected(self):
        self.disconnected.emit()

    def _on_error(self, error):
        error_msg = self.socket.errorString()
        self.error_occurred.emit(error_msg)

    def _on_data_received(self):
        """主线程：只负责把 TCP 流按照行切分，然后丢给 worker"""
        data = self.socket.readAll().data().decode('utf-8', errors='ignore')
        self.buffer += data

        while '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\n', 1)
            if line.strip():
                self.json_line_received.emit(line.strip())

    def _send_command(self, command_dict):
        if self.socket.state() != QAbstractSocket.ConnectedState:
            # print("未连接到树莓派")
            return False

        try:
            json_data = json.dumps(command_dict, ensure_ascii=False)
            self.socket.write(json_data.encode('utf-8'))
            self.socket.write(b'\n')
            self.socket.flush()
            return True
        except Exception as e:
            # print(f"发送命令失败: {e}")
            return False

    @Slot(dict)
    def _cache_motor_response(self, response):
        if response.get('device') != 'motor':
            return
        self._cache_motor_data(response.get('data'))

    def _cache_motor_data(self, data):
        if not isinstance(data, dict):
            return

        if 'id' in data:
            self._cache_single_motor_feedback(data['id'], data)
            return

        for motor_id, value in data.items():
            if isinstance(value, dict):
                feedback = dict(value)
                feedback.setdefault('id', motor_id)
                self._cache_single_motor_feedback(feedback['id'], feedback)

    def _cache_single_motor_feedback(self, motor_id, feedback):
        self.last_motor_feedback[motor_id] = dict(feedback)
        temperature = feedback.get('temperature_c', feedback.get('temperature'))
        if temperature is not None:
            temperature = float(temperature)
            self.last_motor_temperatures[motor_id] = temperature
            self.motor_temperature_received.emit(motor_id, temperature)

    # ==================== 测距仪控制接口（通过信号控制 worker） ====================

    def setLaserRudeDistance(self, distance):
        self.set_rude_distance_signal.emit(float(distance))

    def resetLaserProcessor(self):
        self.reset_laser_signal.emit()

    # 如果你真的需要 buffer 内容，可以再设计一个异步接口

    # ======== 电机控制命令（和原来一样） ========

    def addMotor(
        self,
        motor_id,
        canid,
        Kp=0x0A,
        Kd=0x02,
        currency=2,
        velocity=1,
        pole_pairs=21,
        model='HO7213',
    ):
        motor_config = {
            'model': model,
            'canid': canid,
            'Kp': Kp,
            'Kd': Kd,
            'velocity': velocity,
            'currency': currency,
            'pole_pairs': pole_pairs,
        }
        self.motors[motor_id] = motor_config

        command = {
            'device': 'motor',
            'action': 'add_motor',
            'motor_id': motor_id,
            'config': motor_config
        }
        return self._send_command(command)

    def motorInit(self, motor_id):
        command = {
            'device': 'motor',
            'action': 'init',
            'motor_id': motor_id
        }
        return self._send_command(command)

    def initAllMotors(self):
        command = {'device': 'motor', 'action': 'init_all'}
        return self._send_command(command)

    def motorEnable(self, motor_id):
        command = {'device': 'motor', 'action': 'enable', 'motor_id': motor_id}
        return self._send_command(command)

    def motorDisable(self, motor_id):
        command = {'device': 'motor', 'action': 'disable', 'motor_id': motor_id}
        return self._send_command(command)

    def enableAllMotors(self):
        command = {'device': 'motor', 'action': 'enable_all'}
        return self._send_command(command)

    def disableAllMotors(self):
        command = {'device': 'motor', 'action': 'disable_all'}
        return self._send_command(command)

    def positionControl(self, motor_id):
        command = {
            'device': 'motor',
            'action': 'position_control',
            'motor_id': motor_id
        }
        return self._send_command(command)
    
    def velocityControl(self, motor_id):
        command = {
            'device': 'motor',
            'action': 'velocity_control',
            'motor_id': motor_id
        }
        return self._send_command(command)

    def setZero(self, motor_id):
        command = {
            'device': 'motor',
            'action': 'set_zero',
            'motor_id': motor_id
        }
        return self._send_command(command)

    def setAllMotorsZero(self):
        command = {'device': 'motor', 'action': 'set_all_zero'}
        return self._send_command(command)

    def moveToPosition(self, motor_id, position):
        command = {
            'device': 'motor',
            'action': 'move_to_position',
            'motor_id': motor_id,
            'position': position
        }
        return self._send_command(command)

    def moveWithVelocity(self, motor_id, velocity, current=None):
        command = {
            'device': 'motor',
            'action': 'move_with_velocity',
            'motor_id': motor_id,
            'velocity': velocity
        }
        if current is not None:
            command['current'] = current
        return self._send_command(command)

    def setMotorVelocity(self, motor_id, velocity, current=None):
        return self.moveWithVelocity(motor_id, velocity, current)

    def setMotorParams(self, motor_id, velocity=None, currency=None, Kp=None, Kd=None):
        params = {}
        if velocity is not None:
            params['velocity'] = velocity
        if currency is not None:
            params['currency'] = currency
        if Kp is not None:
            params['Kp'] = Kp
        if Kd is not None:
            params['Kd'] = Kd

        command = {
            'device': 'motor',
            'action': 'set_params',
            'motor_id': motor_id,
            'params': params
        }
        return self._send_command(command)

    def getMotorStatus(self, motor_id):
        command = {
            'device': 'motor',
            'action': 'get_status',
            'motor_id': motor_id
        }
        return self._send_command(command)

    def getAllMotorsStatus(self):
        command = {'device': 'motor', 'action': 'get_all_status'}
        return self._send_command(command)

    def requestMotorTemperature(self, motor_id):
        return self.getMotorStatus(motor_id)

    def requestAllMotorTemperatures(self):
        return self.getAllMotorsStatus()

    def getMotorTemperature(self, motor_id, default=None):
        return self.last_motor_temperatures.get(motor_id, default)

    def getAllMotorTemperatures(self):
        return dict(self.last_motor_temperatures)

    def getLastMotorFeedback(self, motor_id=None):
        if motor_id is None:
            return {mid: dict(data) for mid, data in self.last_motor_feedback.items()}
        data = self.last_motor_feedback.get(motor_id)
        return dict(data) if data else None
    
    def openXyControl(self):
        command = {
            'device': 'motor',
            'action': 'open_xy_control'
        }
        return self._send_command(command)
    
    def closeXyControl(self):
        command = {
            'device': 'motor',
            'action': 'close_xy_control'
        }
        return self._send_command(command)

    def startTracking(self):
        """请求RK3588启动本地PSD闭环。"""
        return self._send_command({
            'device': 'tracking',
            'action': 'start'
        })

    def stopTracking(self):
        """请求RK3588停止本地PSD闭环。"""
        return self._send_command({
            'device': 'tracking',
            'action': 'stop'
        })

    def getTrackingStatus(self):
        return self._send_command({
            'device': 'tracking',
            'action': 'status'
        })

    def benchmarkTrackingRates(self, duration_s=2.0):
        """测试PSD、电机的极限速率及按配置限速的并行速率。"""
        return self._send_command({
            'device': 'tracking',
            'action': 'benchmark_rates',
            'duration_s': float(duration_s),
        })

    def getLinkMetrics(self):
        """查电机 CANFD 链路的实时体检结果。

        返回最近一个 1 秒窗口里每轴实际收到的帧率、反馈龄期、串口积压、
        写超时/丢弃、适配器节流复位次数等。用来确认实际链路频率。
        """
        return self._send_command({
            'device': 'tracking',
            'action': 'get_link_metrics',
        })

    def setLinkSampling(self, on=True):
        """手动开关电机链路保活采样。

        on=True：用当前实测位置原地保持，之后链路按配置速率持续收发，
                 不必启动 PSD 跟踪也能拿到连续反馈（安全，不会让电机动作）。
        on=False：停掉保活，回到"只在有跟踪目标时才发"。
        """
        return self._send_command({
            'device': 'tracking',
            'action': 'set_link_sampling',
            'on': bool(on),
        })

    def testCanfdAutoUpload(self, duration_s=1.0):
        """诊断开启命令应答，以及开启/保持后的空闲主动上传。"""
        return self._send_command({
            'device': 'tracking',
            'action': 'test_canfd_auto_upload',
            'duration_s': float(duration_s),
        })

    def testMotorPositionLoop(
        self,
        step_degrees=(0.05, 0.10, 0.20),
        repeats=3,
        duration_s=1.2,
        command_rate_hz=100.0,
    ):
        """固定频率、多幅值、重复测量HO7213位置环响应。"""
        return self._send_command({
            'device': 'tracking',
            'action': 'test_motor_position_loop',
            'step_degrees': [float(value) for value in step_degrees],
            'repeats': int(repeats),
            'duration_s': float(duration_s),
            'command_rate_hz': float(command_rate_hz),
        })

    def testMotorPositionLoopOnce(
        self,
        step_degrees=(0.05, 0.10, 0.20),
        repeats=3,
        duration_s=1.2,
    ):
        """每个阶跃仅发送一帧目标，由HO7213 CANFD自动上传反馈。"""
        return self._send_command({
            'device': 'tracking',
            'action': 'test_motor_position_loop_once',
            'step_degrees': [float(value) for value in step_degrees],
            'repeats': int(repeats),
            'duration_s': float(duration_s),
        })

    def stopMotorPositionLoopTest(self):
        return self._send_command({
            'device': 'tracking',
            'action': 'stop_motor_position_loop_test',
        })

    # ======== 测距仪命令（这些只发 JSON 不耗时） ========

    def laserEnterControlMode(self):
        command = {'device': 'laser', 'action': 'enter_control_mode'}
        return self._send_command(command)

    def laserOn(self, delay_seconds=10):
        command = {'device': 'laser', 'action': 'laser_on', 'delay': delay_seconds}
        return self._send_command(command)

    def laserOff(self, delay_seconds=5):
        command = {'device': 'laser', 'action': 'laser_off', 'delay': delay_seconds}
        return self._send_command(command)

    def laserSfc(self, sfc=350):
        command = {'device': 'laser', 'action': 'laser_sfc', 'sfc': sfc}
        return self._send_command(command)
    
    def laserLfc(self, lfc=320):
        command = {'device': 'laser', 'action': 'laser_lfc', 'lfc': lfc}
        return self._send_command(command)

    def laserTargetNum(self, tar=2):
        command = {'device': 'laser', 'action': 'laser_target_num', 'tar': tar}
        return self._send_command(command)

    def laserQueryTemperature(self):
        command = {'device': 'laser', 'action': 'query_temperature'}
        return self._send_command(command)

    def laserSetCom1Temperature(self, value):
        command = {'device': 'laser', 'action': 'set_com1_temp', 'value': value}
        return self._send_command(command)

    def laserSetCom2Temperature(self, value):
        command = {'device': 'laser', 'action': 'set_com2_temp', 'value': value}
        return self._send_command(command)

    def laserIncreaseCurrent(self, level):
        command = {'device': 'laser', 'action': 'increase_current', 'level': level}
        return self._send_command(command)

    def laserDecreaseCurrent(self, level):
        command = {'device': 'laser', 'action': 'decrease_current', 'level': level}
        return self._send_command(command)

    def laserSetThresholdRef(self, value):
        command = {'device': 'laser', 'action': 'set_threshold_ref', 'value': value}
        return self._send_command(command)

    def laserSetThresholdSig(self, value):
        command = {'device': 'laser', 'action': 'set_threshold_sig', 'value': value}
        return self._send_command(command)
    
    def laserSetSig(self, value):
        command = {'device': 'laser', 'action': 'set_sig', 'value': value}
        return self._send_command(command)

    def laserEnterAdjustMode(self):
        command = {'device': 'laser', 'action': 'enter_adjust_mode'}
        return self._send_command(command)

    def laserMeasureDistanceMode(self):
        command = {'device': 'laser', 'action': 'measure_distance_mode'}
        return self._send_command(command)

    def laserStopMeasurement(self):
        command = {'device': 'laser', 'action': 'stop_measurement'}
        return self._send_command(command)

    def laserDebugMode(self, enable=True):
        command = {'device': 'laser', 'action': 'debug_mode', 'enable': enable}
        return self._send_command(command)

    def laserGetStatus(self):
        command = {'device': 'laser', 'action': 'get_status'}
        return self._send_command(command)

    def startDataStream(self):
        command = {'device': 'laser', 'action': 'start_data_stream'}
        return self._send_command(command)

    def stopDataStream(self):
        command = {'device': 'laser', 'action': 'stop_data_stream'}
        return self._send_command(command)

    # ======== 视频控制命令（JSON） ========

    def startVideoStream(self, port=5000, width=1280, height=480, fps=30):
        cmd = {
            "device": "video",
            "action": "start_stream",
            "port": port,
            "width": width,
            "height": height,
            "fps": fps
        }
        return self._send_command(cmd)

    def stopVideoStream(self):
        cmd = {"device": "video", "action": "stop_stream"}
        return self._send_command(cmd)


# ====================== 视频解码工作线程 ======================

class VideoDecodeWorker(QObject):
    """在独立线程中解码 UDP 视频帧"""

    frame_decoded = Signal(object)   # 发送 numpy 数组
    fps_updated = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.buffer = bytearray()
        self._frame_count = 0
        self._last_time = time.time()

    @Slot(bytes)
    def process_datagram(self, datagram: bytes):
        """主线程把 datagram 丢进来，在这里解码"""
        self.buffer.extend(datagram)

        try:
            enc = pickle.loads(bytes(self.buffer))
            frame = cv2.imdecode(enc, cv2.IMREAD_COLOR)
            self.buffer.clear()

            if frame is not None:
                self.frame_decoded.emit(frame)

                self._frame_count += 1
                now = time.time()
                if now - self._last_time >= 1.0:
                    fps = self._frame_count / (now - self._last_time)
                    self._frame_count = 0
                    self._last_time = now
                    self.fps_updated.emit(fps)

        except Exception:
            # 未解出完整一帧，继续积累
            pass


# ====================== UDP 视频控件（主线程） ======================

class UdpVideoWidget(QWidget):
    """
    完全异步版 UDP 视频显示控件
    - 主线程只负责收 UDP 包 + 显示图像
    - 解码工作在线程中完成
    """

    datagram_received = Signal(bytes)

    def __init__(self, listen_ip="0.0.0.0", listen_port=5000, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"UDP Video {listen_ip}:{listen_port}")

        self.label = QLabel("等待视频...", alignment=Qt.AlignCenter)
        self.label.setMinimumSize(640, 480)
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

        # UDP 套接字
        self.sock = QUdpSocket(self)
        self.sock.bind(QHostAddress.Any, listen_port)
        self.sock.readyRead.connect(self._on_ready_read)

        self.last_frame = None

        # ===== 视频解码线程 =====
        self.video_thread = QThread(self)
        self.video_worker = VideoDecodeWorker()
        self.video_worker.moveToThread(self.video_thread)

        # 主线程 -> 解码线程
        self.datagram_received.connect(
            self.video_worker.process_datagram,
            Qt.QueuedConnection
        )

        # 解码线程 -> 主线程
        self.video_worker.frame_decoded.connect(self._on_frame_decoded)
        self.video_worker.fps_updated.connect(self._on_fps_updated)

        self.video_thread.start()

        # 定时刷新 UI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_view)
        self.timer.start(30)

    def _on_ready_read(self):
        """主线程：只负责把 datagram 读出来丢给 worker"""
        while self.sock.hasPendingDatagrams():
            size = self.sock.pendingDatagramSize()
            datagram, _, _ = self.sock.readDatagram(size)
            self.datagram_received.emit(datagram)

    @Slot(object)
    def _on_frame_decoded(self, frame):
        """接收已解码 frame（numpy 数组）"""
        self.last_frame = frame

    @Slot(float)
    def _on_fps_updated(self, fps):
        self.setWindowTitle(f"UDP Video - FPS: {fps:.1f}")

    def _update_view(self):
        if self.last_frame is None:
            return
        frame = self.last_frame
        h, w, ch = frame.shape
        qimg = QImage(frame.data, w, h, ch * w, QImage.Format_BGR888)
        self.label.setPixmap(QPixmap.fromImage(qimg).scaled(
            self.label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        ))

    def closeEvent(self, event):
        self.sock.close()
        self.video_thread.quit()
        self.video_thread.wait()
        event.accept()



# ====================== 示例 main ======================

if __name__ == '__main__':
    app = QApplication(sys.argv)

    controller = RobotNetworkController(host='192.168.1.100', port=8888)

    # 示范：连接各种信号（你可以根据自己需要改）
    def on_connected():
        print("=== 连接成功 ===")
        controller.setLaserRudeDistance(5000)

        controller.addMotor('motor1', '01', Kp=0x0A, Kd=0x02, velocity=10)
        controller.addMotor('motor2', '02', Kp=0x0A, Kd=0x02, velocity=10)
        controller.initAllMotors()
        controller.enableAllMotors()
        controller.positionControl('motor1')
        controller.positionControl('motor2')

        controller.laserEnterControlMode()
        controller.laserOn(10)
        controller.laserMeasureDistanceMode()

    def on_motor_response(resp):
        if resp.get('device') == 'motor':
            print("[电机响应]", resp)
        else:
            print("[其他响应]", resp)

    def on_laser_raw(line):
        # 建议这里也不要每行都 print，必要时可以采样打印
        # print("[激光原始]", line)
        pass

    def on_laser_processed(result):
        print(f"[激光处理] 距离: {result['distance']:.3f} mm | "
              f"ng: {result['ng']} | fr1: {result['fr1']} MHz | "
              f"组数: {result['groups_processed']}")

    def on_laser_error(line):
        print("[激光错误]", line)

    def on_error(msg):
        print("[错误]", msg)

    controller.connected.connect(on_connected)
    controller.motor_response_received.connect(on_motor_response)
    controller.error_occurred.connect(on_error)
    controller.laser_raw_data.connect(on_laser_raw)
    controller.laser_processed_data.connect(on_laser_processed)
    controller.laser_error.connect(on_laser_error)

    # 启动连接
    controller.connectToRaspberryPi()

    # 打开视频窗口测试（可选）
    video_widget = UdpVideoWidget(listen_port=5000)
    video_widget.show()

    sys.exit(app.exec())
