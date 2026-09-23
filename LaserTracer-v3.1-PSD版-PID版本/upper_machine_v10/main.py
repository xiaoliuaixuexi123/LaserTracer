import sys
import time
import importlib.util
from pathlib import Path
from PySide6.QtCore import QTimer
from pyside_to_raspberrypi import RobotNetworkController, UdpVideoWidget
from PySide6.QtWidgets import (
    QMainWindow,
    QApplication,
    QVBoxLayout,
    QLineEdit,
    QLabel,
    QPushButton,
    QComboBox,
)
from Ui_mainwindow import Ui_MainWindow
import numpy as np
from laser_serial_worker import LaserSerialWorker
from laser_serial_controller import LaserSerialController
from serial.tools import list_ports
from coordinate_3d_view import Coordinate3DView
from fsdistance import LaserDataProcessor
from wMPS_receiver import create_wmps_receiver_widget
from wmps_distance_constraint import (
    FEMTOSECOND_DISTANCE_ORIGIN_MM,
    constrain_point_to_distance_sphere,
    load_transmitter_origins_from_extpara_file,
    wmps_global_to_femtosecond,
)

GUIDE_MODE_SINGLE = "single"
GUIDE_MODE_MULTI = "multi"
GUIDE_MODE_TEXT = {
    GUIDE_MODE_SINGLE: "单点引导",
    GUIDE_MODE_MULTI: "多点引导",
}





class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("飞秒激光跟踪仪 - V10位置跟踪版")
        self.motor_velocity_mode.setText("速度模式（手动调试）")
        self.motor_velocity_mode.setToolTip(
            "V10的PSD闭环使用位置模式；此按钮仅保留给手动速度模式调试"
        )
        self.max_rate_test_button = QPushButton("测试PSD和电机频率（约8秒）", self.tab_6)
        self.max_rate_test_button.setToolTip(
            "初始化并使能双轴后点击；自动重发原位置目标取得新反馈，"
            "依次测PSD极限、设定频率并行验证、电机极限和并行极限"
        )
        self.verticalLayout_3.addWidget(self.max_rate_test_button)
        # self.controller = RobotNetworkController(host=self.socket_connection.text(), port="8888")
        self.controller = None
        self.pending_pitch = None
        self.pending_yaw = None
        self.position_counter = 0
        self.last_collected_wmps_xyz = None
        self.last_collected_wmps_mode = None
        self.wmps_center_solver = None
        self.wmps_center_params = None
        self.wmps_center_warn_rms_mm = 1.0
        self.last_wmps_guidance_source = ""
        self.last_wmps_guidance_diag = None
        self.last_wmps_transmitter_ids = ()
        self.last_femtosecond_distance_mm = None
        self.last_final_femtosecond_distance_mm = None
        self.distance_constraint_origin_mm = FEMTOSECOND_DISTANCE_ORIGIN_MM.copy()
        self.laser_measurement_wmps_xyz = None
        self.laser_measurement_wmps_mode = None
        self.laser_measurement_wmps_source = ""
        self.last_distance_constrained_wmps_xyz = None
        self.last_distance_constrained_femtosecond_xyz = None
        self.last_four_marker_corner_wmps_xyz = None
        self.last_solution_distance_mm = None
        self.wmps_transmitter_positions_global_mm = {}
        self.wmps_transmitter_positions_source = ""
        self.laser_processor = LaserDataProcessor()
        self.coordinate_result.setReadOnly(True)
        self.coordinate_result.setPlaceholderText("等待四靶标与飞秒测距结果")
        self.bind()

        # ===== wMPS 接收器控件 =====
        self.wmps_receiver, self.wmps_controller = create_wmps_receiver_widget()
        wmps_layout = QVBoxLayout(self.wMPS_widget)
        wmps_layout.setContentsMargins(0, 0, 0, 0)
        wmps_layout.addWidget(self.wmps_receiver)
        self.wmps_controller.transmitter_positions_changed.connect(
            self._on_wmps_transmitter_positions_changed
        )

        # ===== 全向测距原点坐标系 3D 显示 =====
        self._init_coordinate_3d_view()

        # ===== wMPS guidance controls =====
        self.collect_button = QPushButton("采集", self.tab_10)
        self.collect_button.setStyleSheet("font-size: 20pt;")
        self.horizontalLayout_31.addWidget(self.collect_button)
        self.collect_button.clicked.connect(self._collect_current_value)

        self.guide_mode_label = QLabel("引导模式", self.tab_10)
        self.guide_mode_label.setStyleSheet("font-size: 14pt;")
        self.horizontalLayout_31.addWidget(self.guide_mode_label)

        self.guide_mode_combo = QComboBox(self.tab_10)
        self.guide_mode_combo.setObjectName("guide_mode_combo")
        self.guide_mode_combo.addItems([
            "请选择模式",
            GUIDE_MODE_TEXT[GUIDE_MODE_SINGLE],
            GUIDE_MODE_TEXT[GUIDE_MODE_MULTI],
        ])
        self.guide_mode_combo.setStyleSheet("font-size: 14pt;")
        self.guide_mode_combo.setFixedWidth(140)
        self.horizontalLayout_31.addWidget(self.guide_mode_combo)
        self.guide_mode_combo.currentTextChanged.connect(self._on_wmps_guide_mode_changed)

        self.guide_button = QPushButton("引导", self.tab_10)
        self.guide_button.setStyleSheet("font-size: 20pt;")
        self.horizontalLayout_31.addWidget(self.guide_button)
        self.guide_button.clicked.connect(self._guide_to_collected_value)

        self.realtime_guide_button = QPushButton("实时引导", self.tab_10)
        self.realtime_guide_button.setStyleSheet("font-size: 20pt;")
        self.horizontalLayout_31.addWidget(self.realtime_guide_button)
        self.realtime_guide_button.clicked.connect(self.toggle_realtime_wmps_guidance)

        # ===== 位置序列自动运动 =====
        self.auto_timer = QTimer()
        self.auto_timer.timeout.connect(self._moveToNextPoint)

        self.auto_points = []     # [(pitch, yaw), ...]
        self.auto_index = 0
        self.auto_running = False

        # wMPS realtime closed-loop guidance.
        self.realtime_guide_enabled = False
        self.realtime_guide_interval_ms = 300
        self.realtime_guide_deadband_deg = 0.01
        self.realtime_guide_timer = QTimer(self)
        self.realtime_guide_timer.setInterval(self.realtime_guide_interval_ms)
        self.realtime_guide_timer.timeout.connect(self._run_realtime_wmps_guidance)
        self._last_realtime_guide_cmd = None
        self._realtime_guide_log_time = 0.0

        # Laser measurement automation: MEA,1 -> 1 -> LPP -> finalize.
        self.laser_auto_measure_ms = 2000
        self.laser_lpp_query_delay_ms = 500
        self.laser_lpp_retry_delay_ms = 500
        self.laser_lpp_timeout_ms = 3000
        self.laser_waiting_for_lpp = False
        self.laser_auto_stop_timer = QTimer(self)
        self.laser_auto_stop_timer.setSingleShot(True)
        self.laser_auto_stop_timer.timeout.connect(self._stop_measurement_and_query_lpp)
        self.laser_lpp_timeout_timer = QTimer(self)
        self.laser_lpp_timeout_timer.setSingleShot(True)
        self.laser_lpp_timeout_timer.timeout.connect(self._finalize_laser_after_lpp_timeout)
        self.manual_lpp_query_active = False
        self.last_lpp_mm = None
        self.manual_lpp_timeout_timer = QTimer(self)
        self.manual_lpp_timeout_timer.setSingleShot(True)
        self.manual_lpp_timeout_timer.timeout.connect(self._manual_lpp_timeout)

            # ===== 新增：测距仪 =====
        self.laser_serial = None       # LaserSerialWorker
        self.laser_cmd = None          # LaserSerialController
        self.scan_serial_ports()
        self.fsDistancePb.clicked.connect(self.connect_laser_serial)

        self.lpp_value_display = QLineEdit(self.tab_5)
        self.lpp_value_display.setReadOnly(True)
        self.lpp_value_display.setText("LPP: -- mm")
        self.horizontalLayout_6.addWidget(self.lpp_value_display)

        self.rude_distance_mode_combo = QComboBox(self.tab_5)
        self.rude_distance_mode_combo.setObjectName("rude_distance_mode_combo")
        self.rude_distance_mode_combo.addItems(["手动粗测", "wMPS粗测"])
        self.rude_distance_mode_combo.setFixedWidth(110)
        self.horizontalLayout_28.addWidget(self.rude_distance_mode_combo)

    def _init_coordinate_3d_view(self):
        layout = self.tab_8.layout()
        if layout is None:
            layout = QVBoxLayout(self.tab_8)
            layout.setContentsMargins(0, 0, 0, 0)
        self.coordinate_3d_view = Coordinate3DView(self.tab_8)
        layout.addWidget(self.coordinate_3d_view)

        default_extpara = (
            Path(__file__).resolve().parent
            / "内外参"
            / "space_resection_calib.txt"
        )
        try:
            positions = load_transmitter_origins_from_extpara_file(default_extpara)
        except Exception as exc:
            print(f"[3D显示] 默认wMPS外参读取失败: {exc}")
            positions = {}
        self._set_wmps_transmitter_positions(
            positions,
            f"默认标定 {default_extpara.name}" if positions else "",
        )

    def _on_wmps_transmitter_positions_changed(self, payload):
        if isinstance(payload, dict) and "positions" in payload:
            positions = payload.get("positions", {})
            source = payload.get("source", "当前已加载wMPS外参")
        else:
            positions = payload
            source = "当前已加载wMPS外参"
        self._set_wmps_transmitter_positions(positions, source)

    def _set_wmps_transmitter_positions(self, positions, source):
        normalized = {}
        for transmitter_id, position in (positions or {}).items():
            point = np.asarray(position, dtype=float).reshape(-1)
            if point.size < 3 or not np.all(np.isfinite(point[:3])):
                continue
            normalized[int(transmitter_id)] = point[:3].copy()
        self.wmps_transmitter_positions_global_mm = normalized
        self.wmps_transmitter_positions_source = str(source or "")
        self._refresh_coordinate_3d_view()

    def _refresh_coordinate_3d_view(self):
        view = getattr(self, "coordinate_3d_view", None)
        if view is None:
            return
        local_stations = {
            transmitter_id: wmps_global_to_femtosecond(
                position,
                self.distance_constraint_origin_mm,
            )
            for transmitter_id, position
            in self.wmps_transmitter_positions_global_mm.items()
        }
        view.update_scene(
            local_stations,
            self.last_distance_constrained_femtosecond_xyz,
            self.wmps_transmitter_positions_source,
        )

    def scan_serial_ports(self):
        self.fsDistanceCbb.clear()
        for p in list_ports.comports():
            self.fsDistanceCbb.addItem(
                f"{p.device} ({p.description})",
                p.device
            )

    def connect_laser_serial(self):
        port = self.fsDistanceCbb.currentData()
        if not port:
            print("未选择串口")
            return

        # 1. 如果之前连过，先关
        if self.laser_serial:
            self.laser_serial.stop()
            self.laser_serial = None
            self.laser_cmd = None

        # 2. 创建串口 worker
        self.laser_serial = LaserSerialWorker(
            port=port,
            baud=921600
        )

        # 3. 串口 → 激光算法
        self.laser_serial.raw_line.connect(
            self.laser_processor.process_line
        )
        self.laser_serial.error.connect(self.on_laser_error)

        self.laser_serial.start()

        # 4. 创建"指令控制器"
        self.laser_cmd = LaserSerialController(self.laser_serial)

        # 5. 绑定 UI 按钮 → 串口指令
        self._bind_laser_buttons()

        print(f"✓ 测距仪已连接: {port}")

    def _bind_laser_buttons(self):
        if getattr(self, "_laser_buttons_bound", False):
            return
        self._laser_buttons_bound = True

        # Bind once and resolve self.laser_cmd at click time.  Reconnecting the
        # serial port therefore neither stacks duplicate slots nor retains an
        # obsolete controller instance.
        self.CPU1.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.enter_control_mode()
        )
        self.LON.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.laser_on(5)
        )
        self.LFF.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.laser_off(5)
        )
        self.SFC.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.set_sfc(self.SFC_SB.value())
        )
        self.LFC.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.set_lfc(self.LFC_SB.value())
        )
        self.TAR.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.set_target_num(self.TAR_SB.value())
        )
        self.TH1.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.set_threshold_ref(self.TH1_SB.value())
        )
        self.TH2.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.set_threshold_sig(self.TH2_SB.value())
        )
        self.AJT.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.enter_adjust_mode()
        )
        self.SIG.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.set_sig(self.SIG_SB.value())
        )
        self.TEM.clicked.connect(
            lambda: self.laser_cmd and self.laser_cmd.query_temperature()
        )
        self.LPP.clicked.connect(self.query_lpp_value)
        self.fs_start_measurement.clicked.connect(
            self.toggle_measurement
        )

        self.fs_stop_measurement.clicked.connect(
            self.stop_measurement
        )



    def bind(self):
        self.socket_connection_pushButton.clicked.connect(self.socketConnection)
        self.socket_disconnection_pushButton.clicked.connect(self.socketDisconnection)
        self.start_tracking.clicked.connect(self.startPsdTracking)
        self.stop_tracking.clicked.connect(self.stopPsdTracking)
        self.read_currunt_position.clicked.connect(self.save_current_position)
        self.max_rate_test_button.clicked.connect(self.runMaxRateTest)
        self.delete_last_line.clicked.connect(self.delete_last_position_fc)
        self.laser_processor.raw_data_received.connect(self.on_laser_raw_data)
        self.laser_processor.processed_result.connect(self.on_laser_processed)
        self.laser_processor.final_result.connect(self.on_laser_final_result)
        self.laser_processor.lpp_ready.connect(self.on_laser_lpp_ready)
        self.laser_processor.rolling_result.connect(self.on_laser_rolling_result)
        self.laser_processor.error_detected.connect(self.on_laser_error)
        self.get_coordinate.clicked.connect(self.solve_coordinate_once)

    def save_current_position(self):
        try:
            pitch = float(self.pitch_angle.value())
            yaw = float(self.yaw_angle.value())
        except ValueError:
            print("pitch_angle 或 yaw_angle 不是有效浮点数，保存失败。")
            return

        txt_path = Path(__file__).resolve().parent / "current_position.txt"
        with txt_path.open("a", encoding="utf-8") as f:
            f.write(f"{self.position_counter} {pitch} {yaw}\n")
        print(f"已保存当前位置到 {txt_path}: {self.position_counter} {pitch} {yaw}")
        self.position_counter += 1

    def runMaxRateTest(self):
        """测PSD、双轴命令应答及并行运行的实际最大频率。"""
        if self.controller is None:
            self.statusbar.showMessage("下位机未连接，无法测速", 5000)
            return
        self.max_rate_test_button.setEnabled(False)
        self.start_tracking.setEnabled(False)
        if self.controller.benchmarkTrackingRates(duration_s=2.0):
            self.statusbar.showMessage("正在测速：PSD极限、设定并行、电机极限和并行极限各约2秒…")
            return
        self.max_rate_test_button.setEnabled(True)
        self.start_tracking.setEnabled(True)
        self.statusbar.showMessage("测速命令发送失败", 5000)

    def delete_last_position_fc(self):
        txt_path = Path(__file__).resolve().parent / "current_position.txt"

        if not txt_path.exists():
            print("current_position.txt 不存在，无法删除。")
            return

        lines = txt_path.read_text(encoding="utf-8").splitlines()

        if not lines:
            print("文件为空，没有可删除的行。")
            return

        deleted_line = lines.pop()  # 删除最后一行

        with txt_path.open("w", encoding="utf-8") as f:
            for line in lines:
                f.write(line + "\n")

        print(f"已删除最后一行: {deleted_line}")


    # TCP连接
    def socketConnection(self):
        self.controller = RobotNetworkController(host=self.socket_connection.text(), port=8888)

        # 电机
        # 信号绑定
        self.controller.connected.connect(self.on_connected)
        self.controller.motor_response_received.connect(self.on_response)
        self.controller.motor_feedback_received.connect(self.on_motor_feedback)
        self.controller.error_occurred.connect(self.on_error)
        self.controller.disconnected.connect(self.on_disconnected)
        self.add_motors.clicked.connect(self.addMotors)
        self.motor_position_mode.clicked.connect(self.motorPositionMode)
        self.motor_velocity_mode.clicked.connect(self.velocityPositionMode)
        self.set_zero_position.clicked.connect(self.setZeroPosition)
        self.enable_motor.clicked.connect(self.enableMotor)
        self.disable_motor.clicked.connect(self.disableMotor)
        # self.send_motor_message.clicked.connect(self.sendMotorMessage)
        self.send_motor_message.clicked.connect(self._motorValueChanged)
        self.send_timer = QTimer()
        self.send_timer.timeout.connect(self._sendMotorIfPending)
        self.send_timer.start(100)  # 每 100ms 检查一次

        # 遥控加减绑定
        self.up_1.clicked.connect(lambda: self.pitch_angle.setValue(self.pitch_angle.value() + 1))
        self.up__1.clicked.connect(lambda: self.pitch_angle.setValue(self.pitch_angle.value() + 0.1))
        self.up___1.clicked.connect(lambda: self.pitch_angle.setValue(self.pitch_angle.value() + 0.01))
        self.up____1.clicked.connect(lambda: self.pitch_angle.setValue(self.pitch_angle.value() + 0.001))

        self.down_1.clicked.connect(lambda: self.pitch_angle.setValue(self.pitch_angle.value() - 1))
        self.down__1.clicked.connect(lambda: self.pitch_angle.setValue(self.pitch_angle.value() - 0.1))
        self.down___1.clicked.connect(lambda: self.pitch_angle.setValue(self.pitch_angle.value() - 0.01))
        self.down____1.clicked.connect(lambda: self.pitch_angle.setValue(self.pitch_angle.value() - 0.001))

        self.left_1.clicked.connect(lambda: self.yaw_angle.setValue(self.yaw_angle.value() - 1))
        self.left__1.clicked.connect(lambda: self.yaw_angle.setValue(self.yaw_angle.value() - 0.1))
        self.left___1.clicked.connect(lambda: self.yaw_angle.setValue(self.yaw_angle.value() - 0.01))
        self.left____1.clicked.connect(lambda: self.yaw_angle.setValue(self.yaw_angle.value() - 0.001))

        self.right_1.clicked.connect(lambda: self.yaw_angle.setValue(self.yaw_angle.value() + 1))
        self.right__1.clicked.connect(lambda: self.yaw_angle.setValue(self.yaw_angle.value() + 0.1))
        self.right___1.clicked.connect(lambda: self.yaw_angle.setValue(self.yaw_angle.value() + 0.01))
        self.right____1.clicked.connect(lambda: self.yaw_angle.setValue(self.yaw_angle.value() + 0.001))

        # 值变化绑定
        self.pitch_angle.valueChanged.connect(self._motorValueChanged)
        self.yaw_angle.valueChanged.connect(self._motorValueChanged)
    
        # 摇杆绑定
        self.open_xy.clicked.connect(self.controller.openXyControl)
        self.close_xy.clicked.connect(self.controller.closeXyControl)

        # # 测距仪
        # self.controller.laser_raw_data.connect(self.on_laser_raw_data)
        # self.controller.laser_processed_data.connect(self.on_laser_processed)
        # self.controller.laser_final_data.connect(self.on_laser_final_result)
        # self.controller.laser_error.connect(self.on_laser_error)
        # self.CPU1.clicked.connect(self.controller.laserEnterControlMode)
        # self.LON.clicked.connect(lambda: self.controller.laserOn(5))
        # self.LFF.clicked.connect(lambda: self.controller.laserOff(5))
        # self.SFC.clicked.connect(lambda: self.controller.laserSfc(self.SFC_SB.value()))
        # self.LFC.clicked.connect(lambda: self.controller.laserLfc(self.LFC_SB.value()))
        # self.TAR.clicked.connect(lambda: self.controller.laserTargetNum(self.TAR_SB.value()))
        # self.TH1.clicked.connect(lambda: self.controller.laserSetThresholdRef(self.TH1_SB.value()))
        # self.TH2.clicked.connect(lambda: self.controller.laserSetThresholdSig(self.TH2_SB.value()))
        # self.AJT.clicked.connect(self.controller.laserEnterAdjustMode)
        # self.fs_start_measurement.clicked.connect(self.start_measurement)
        # self.fs_stop_measurement.clicked.connect(self.stop_measurement)
        # self.start_stream.clicked.connect(self.controller.startDataStream)
        # self.close_stream.clicked.connect(self.controller.stopDataStream)
        # self.SIG.clicked.connect(lambda: self.controller.laserSetSig(self.SIG_SB.value()))
        # self.TEM.clicked.connect(self.controller.laserQueryTemperature)
        

        self.controller.connectToRaspberryPi()

    # =====camera=====
    def open_camera_clicked(self):
        self.camera_widget1 = BallTrackerWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.camera_widget1)
        self.camera_widget.setLayout(layout)
        self.camera_widget1.start()

    def close_camera_clicked(self):
        self.camera_widget1.stop()

    # 初始化电机
    def addMotors(self):
        self.controller.addMotor(
            'motor1', '01', Kp=0x06, Kd=0xF0, velocity=2, currency=3,
            pole_pairs=21, model='HO7213'
        )  # HO7213，CAN ID 01，俯仰角
        self.controller.addMotor(
            'motor2', '02', Kp=0x10, Kd=0xA0, velocity=2, currency=3,
            pole_pairs=21, model='HO7213'
        )  # HO7213，CAN ID 02，偏航角
        self.controller.initAllMotors()

    # 设置位置模式
    def motorPositionMode(self):
        # 先更新位置环参数；下位机随后会以实时反馈位置作为目标无扰切换。
        self.controller.setMotorParams(
            'motor1', Kp=0x1E, Kd=0xD2, velocity=2, currency=2
        )
        self.controller.setMotorParams(
            'motor2', Kp=0x1C, Kd=0x80, velocity=2, currency=2
        )
        self.controller.positionControl('motor1')
        self.controller.positionControl('motor2')

    def velocityPositionMode(self):
        # PSD速度跟踪禁止由上位机预先切换速度模式。电机在初始化和
        # 使能阶段保持位置模式，第一条有效非零PSD命令出现后，由下位机
        # 完成受控切换。保留这个按钮，但将它改为“恢复位置保持”。
        print("[PSD跟踪] 速度模式由下位机自动管理，当前继续保持位置模式")
        self.statusbar.showMessage(
            "速度模式由PSD下位机自动切换；当前已请求位置保持", 5000
        )
        self.controller.positionControl('motor1')
        self.controller.positionControl('motor2')

    # 设置电机零点（掉电不保存，只能通过485通讯设计掉电保存的）
    def setZeroPosition(self):
        self.controller.setZero('motor1')
        self.controller.setZero('motor2')

    # 电机使能
    def enableMotor(self):
        # 无论此前界面处于什么状态，使能前都显式保持当前位置模式。
        # 不在上位机初始化阶段发送 velocity_control。
        self.controller.positionControl('motor1')
        self.controller.positionControl('motor2')
        self.controller.enableAllMotors()

    # 电机失能
    def disableMotor(self):
        self.controller.disableAllMotors()

    # TCP断开连接
    def socketDisconnection(self):
        self.stop_realtime_wmps_guidance()
        if self.controller is not None:
            self.controller.disconnect()

    # 初始化回到零点或输入到指定位置
    def sendMotorMessage(self):
        self.controller.moveToPosition(motor_id='motor1', position=self.pitch_angle.value())
        self.controller.moveToPosition(motor_id='motor2', position=self.yaw_angle.value())

    def startPsdTracking(self):
        if self.controller is None:
            print("⚠️ 下位机未连接，无法启动PSD跟踪")
            self.statusbar.showMessage("下位机未连接，无法启动PSD跟踪", 5000)
            return
        if self.auto_running:
            self.stopAutoMove()
        if self.controller.startTracking():
            print("=== 已发送PSD跟踪启动命令 ===")
            self.statusbar.showMessage("已发送PSD跟踪启动命令，等待下位机响应", 5000)
        else:
            print("⚠️ PSD跟踪启动命令发送失败")
            self.statusbar.showMessage("PSD跟踪启动命令发送失败", 5000)

    def stopPsdTracking(self):
        if self.controller is None:
            print("⚠️ 下位机未连接，无法停止PSD跟踪")
            self.statusbar.showMessage("下位机未连接，无法停止PSD跟踪", 5000)
            return
        if self.controller.stopTracking():
            print("=== 已发送PSD跟踪停止命令 ===")
            self.statusbar.showMessage("已发送PSD跟踪停止命令", 5000)
        else:
            print("⚠️ PSD跟踪停止命令发送失败")
            self.statusbar.showMessage("PSD跟踪停止命令发送失败", 5000)

    def _motorValueChanged(self):
        # 覆盖式发送缓存：新的值覆盖旧的值
        self.pending_pitch = self.pitch_angle.value()
        self.pending_yaw = self.yaw_angle.value()

    def _sendMotorIfPending(self):
        if self.pending_pitch is None:
            return False
        if self.controller is None:
            return False

        pitch = self.pending_pitch
        yaw = self.pending_yaw
        ok_pitch = self.controller.moveToPosition('motor1', pitch)
        ok_yaw = self.controller.moveToPosition('motor2', yaw)

        if ok_pitch and ok_yaw:
            if self.pending_pitch == pitch and self.pending_yaw == yaw:
                self.pending_pitch = None
                self.pending_yaw = None
            return True
        return False
    
    # def start_measurement(self):
    #     self.controller.worker.laser_processor.rude_distance = int(self.rude_distance1.text())
    #     # print(1)
    #     print(f"设置粗距为: {self.controller.worker.laser_processor.rude_distance} mm")
    #     self.controller.worker.laser_processor.raw_ch1 = []
    #     self.controller.worker.laser_processor.raw_ch2 = []
    #     self.controller.laserMeasureDistanceMode()

    # def stop_measurement(self):
    #     self.controller.laserStopMeasurement()
    #     self.controller.laserStopMeasurement()
    #     # result = self.laser_processor.finalize_results()
    #     # print("最终合成结果：", result)
    #     # self.fs_measurement_result.setText(f"{result['final_ch1_um']:.2f} μm")
    #     self.controller.finalize_laser_signal.emit()   # ★ 触发最终运算
    #     self.controller.worker.laser_processor.current_ng = None
    #     self.controller.worker.laser_processor.current_fr1 = None
    #     self.controller.worker.laser_processor.channel1_values = []
    #     self.controller.worker.laser_processor.channel2_values = []
    #     self.controller.worker.laser_processor.data_line_position = 0
    #     self.controller.worker.laser_processor.channel1_avg_values = []
    #     self.controller.worker.laser_processor.channel2_avg_values = []
        
    # 连接、收到信号、断开连接、报错处理
    def on_connected(self):
        print("=== 连接成功 ===")

    def on_motor_feedback(self, response):
        """用下位机主动推送的最新样本刷新界面，不逐帧打印。"""
        payload = response.get('data') or {}
        for motor_id, feedback in payload.items():
            if not isinstance(feedback, dict) or 'position' not in feedback:
                continue
            position = feedback['position']
            if motor_id == 'motor1':
                self.current_x = position
                self.actual_pitch_angle.setText(str(position))
            elif motor_id == 'motor2':
                self.current_y = position
                self.actual_yaw_angle.setText(str(position))

    def on_response(self, response):
        print(f"=== 收到响应 ===")
        print(f"设备: {response.get('device')}")
        print(f"动作: {response.get('action')}")
        print(f"状态: {response.get('status')}")
        if response.get('data'):
            data = response.get('data')
            print(f"数据: {data}")
            if 'id' in data:
                if data.get('id') == 'motor2':
                    self.current_y = data['position']
                    self.actual_yaw_angle.setText(str(data['position']))
                elif data.get('id') == 'motor1':
                    self.current_x = data['position']
                    self.actual_pitch_angle.setText(str(data['position']))
        if response.get('message'):
            print(f"消息: {response.get('message')}")
        if (
            response.get('device') == 'motor'
            and response.get('action') == 'get_status'
            and isinstance(response.get('data'), dict)
        ):
            age_ms = response['data'].get('feedback_age_ms')
            if isinstance(age_ms, (int, float)) and age_ms > 50.0:
                self.statusbar.showMessage(
                    f"{response['data'].get('id', '电机')}反馈已过期"
                    f"（{age_ms:.1f} ms）；测速或跟踪启动时将自动尝试更新反馈",
                    8000,
                )
        if response.get('device') == 'tracking':
            action = response.get('action')
            status = response.get('status')
            message = response.get('message', '')
            if status == 'success' and action == 'start':
                self.start_tracking.setEnabled(False)
                self.stop_tracking.setEnabled(True)
                self.statusbar.showMessage(f"PSD跟踪已启动：{message}")
            elif status == 'success' and action == 'stop':
                self.start_tracking.setEnabled(True)
                self.stop_tracking.setEnabled(False)
                self.statusbar.showMessage(f"PSD跟踪已停止：{message}", 5000)
            elif action == 'benchmark_rates':
                self.max_rate_test_button.setEnabled(True)
                self.start_tracking.setEnabled(True)
                if status == 'success':
                    data = response.get('data') or {}
                    psd_only = data.get('psd_only', {})
                    motor_only = data.get('motor_only', {})
                    combined = data.get('combined', {})
                    configured = data.get('configured', {})
                    configured_psd = configured.get('psd', {})
                    configured_motor = configured.get('motor', {})
                    combined_psd = combined.get('psd', {})
                    combined_motor = combined.get('motor', {})
                    if not configured:
                        summary = "下位机未返回设定频率并行结果，请部署v22下位机与新配置"
                    else:
                        summary = (
                            f"设定并行PSD/双轴TX/RX "
                            f"{configured_psd.get('sample_hz', 0):.1f}/"
                            f"{configured_motor.get('tx_pair_hz', 0):.1f}/"
                            f"{configured_motor.get('completed_pair_hz', 0):.1f}Hz"
                            f"（{'达标' if configured.get('meets_target') else '未达标'}）；"
                            f"极限PSD {psd_only.get('sample_hz', 0):.1f}Hz，"
                            f"电机发/收 {motor_only.get('tx_pair_hz', 0):.1f}/"
                            f"{motor_only.get('completed_pair_hz', 0):.1f}Hz，"
                            f"并行 {combined_psd.get('sample_hz', 0):.1f}/"
                            f"{combined_motor.get('completed_pair_hz', 0):.1f}Hz"
                        )
                    print(summary)
                    self.statusbar.showMessage(summary, 20000)
                else:
                    self.statusbar.showMessage(f"最大频率测试失败：{message}", 10000)
            elif status in ('error', 'cancelled'):
                self.start_tracking.setEnabled(True)
                self.stop_tracking.setEnabled(True)
                self.statusbar.showMessage(f"PSD跟踪失败：{message}", 10000)
        print()

    def on_error(self, error_msg):
        print(f"错误: {error_msg}")

    def on_disconnected(self):
        self.stop_realtime_wmps_guidance()
        self.max_rate_test_button.setEnabled(True)
        self.start_tracking.setEnabled(True)
        print("=== 断开连接成功 ===")

    def on_laser_raw_data(self, data_line):
        """测距仪原始数据 - 实时显示"""
        print(f"[激光原始] {data_line}")

    def on_laser_processed(self, result):
        """测距仪处理结果 - 实时更新"""
        # print(f"[激光处理] 距离1: {result['distance_mm']:.3f} mm | "
        #       f"lpp: {result['lpp_mm']:.3f} mm | "
        #       f"ng: {result['ng']} | "
        #       f"fr1: {result['fr1']} MHz | "
        #       f"组数: {result['group_index']}")
        # self.measurement_result.setText(f'{result["distance_mm"]:.3f}')

        print(f"[激光处理] 距离1: {result['distance_mm']:.3f} mm | "
              f"lpp: {result['lpp_mm']:.3f} mm | "
              f"ng: {result['ng']} | "
              f"fr1: {result['fr1']} MHz | "
              f"组数: {result['group_index']}")
        self.last_femtosecond_distance_mm = float(result["distance_mm"])
        self.measurement_result.setText(f'{result["distance_mm"]:.3f}')        

    def on_laser_final_result(self, final_result):
        """测距仪最终结果"""
        # print(f"[激光最终结果] 距离1: {final_result['final_mean_mm']:.3f} mm ")
        # self.measurement_result.setText(f'{final_result["final_mean_mm"]:.3f}')
        distance_mm = float(final_result["final_mean_mm"])
        print(f"[激光最终结果] 距离1: {distance_mm:.3f} mm ")
        self.last_femtosecond_distance_mm = distance_mm
        self.last_final_femtosecond_distance_mm = distance_mm
        self.measurement_result.setText(f'{distance_mm:.3f}')
        self._apply_laser_distance_constraint(distance_mm)

    def on_laser_error(self, error_line):
        """测距仪错误数据"""
        print(f"[激光错误] {error_line}")

    def _initAutoPoints(self):
        """
        你指定的二维转台位置序列
        """
        self.auto_points = [
            (0,   0),
            (-1.766,  28.209),
            (-6.155,   -4.999),
        ]

    def startAutoMove(self):
        if self.auto_running:
            return

        self._initAutoPoints()

        if len(self.auto_points) == 0:
            print("⚠️ 自动运动点为空")
            return

        print("=== 自动二维转台：开始按点运动 ===")

        self.auto_index = 0
        self.auto_running = True

        # 每 1 秒走一个点（你可以改）
        self.auto_timer.start(3000)


    def _moveToNextPoint(self):
        if not self.auto_running:
            return

        # === 核心改动：到末尾就回到 0 ===
        if self.auto_index >= len(self.auto_points):
            self.auto_index = 0   # ← 不停止，回到第一个点

        pitch, yaw = self.auto_points[self.auto_index]

        print(f"→ 移动到点 {self.auto_index}: pitch={pitch}, yaw={yaw}")

        # 更新 UI，复用你现有发送机制
        self.pitch_angle.setValue(pitch)
        self.yaw_angle.setValue(yaw)

        self.auto_index += 1


    def stopAutoMove(self):
        if not self.auto_running:
            return

        print("=== 自动二维转台：已停止 ===")
        self.auto_running = False
        self.auto_timer.stop()


    def toggle_measurement(self):
        """Measurement button: MEA,1 -> timed stop -> LPP -> finalize."""
        btn = self.fs_start_measurement
        current_text = btn.text()

        if current_text == "\u6d4b\u91cf":
            if self._do_start_measurement():
                btn.setText("\u6682\u505c")
        elif current_text == "\u6682\u505c":
            self.laser_processor.set_measurement_active(False)
            btn.setText("\u7ee7\u7eed")
        elif current_text == "\u7ee7\u7eed":
            self.laser_processor.set_measurement_active(True)
            btn.setText("\u6682\u505c")

    def _is_wmps_rude_distance_mode(self):
        combo = getattr(self, "rude_distance_mode_combo", None)
        return combo is not None and combo.currentText().strip() == "wMPS粗测"

    def _clear_laser_measurement_wmps_snapshot(self):
        self.laser_measurement_wmps_xyz = None
        self.laser_measurement_wmps_mode = None
        self.laser_measurement_wmps_source = ""

    def _snapshot_wmps_for_laser_measurement(self, log_prefix="[测距约束]", required=False):
        self._clear_laser_measurement_wmps_snapshot()
        # Coordinate fusion is deliberately stricter than ordinary guidance:
        # it must always use all four cooperative-target receivers.
        coords = self._read_multi_point_wmps_xyz(
            log_prefix,
            suppress_errors=not required,
        )
        if coords is None:
            if required:
                print(f"{log_prefix} 四靶标角锥坐标尚不可用")
            return None

        coords = tuple(float(v) for v in coords)
        self.laser_measurement_wmps_xyz = coords
        self.laser_measurement_wmps_mode = GUIDE_MODE_MULTI
        self.laser_measurement_wmps_source = self.last_wmps_guidance_source
        self.last_collected_wmps_xyz = coords
        self.last_collected_wmps_mode = GUIDE_MODE_MULTI
        self.last_four_marker_corner_wmps_xyz = coords
        self.wmps_receiver.set_collected_coords(*coords)
        return coords

    def _wmps_rude_distance_from_coords(self, coords):
        point = np.asarray(coords, dtype=float).reshape(3)
        origin = np.asarray(self.distance_constraint_origin_mm, dtype=float).reshape(3)
        return float(np.linalg.norm(point - origin))

    def _resolve_laser_rude_distance(self):
        if self._is_wmps_rude_distance_mode():
            coords = self._snapshot_wmps_for_laser_measurement("[wMPS粗测]", required=True)
            if coords is None:
                return None
            rude_distance = self._wmps_rude_distance_from_coords(coords)
            self.rude_distance1.setText(f"{rude_distance:.3f}")
            x, y, z = coords
            origin = self.distance_constraint_origin_mm
            print(
                f"[wMPS粗测] 粗测值={rude_distance:.3f} mm | "
                f"wMPS=({x:.4f}, {y:.4f}, {z:.4f}) | "
                f"origin=({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f})"
            )
            return rude_distance

        try:
            rude_distance = float(self.rude_distance1.text())
        except ValueError:
            print(f"[Laser] invalid rude distance: {self.rude_distance1.text()}")
            return None

        if not np.isfinite(rude_distance) or rude_distance < 0.0:
            print(f"[Laser] invalid rude distance: {self.rude_distance1.text()}")
            return None

        self._snapshot_wmps_for_laser_measurement("[测距约束]", required=False)
        print(f"[手动粗测] 粗测值={rude_distance:.3f} mm")
        return rude_distance

    def _append_laser_distance_constraint_log(
        self,
        raw_xyz,
        constrained_xyz,
        local_xyz,
        distance_mm,
        raw_radius_mm,
        correction_mm,
    ):
        txt_path = Path(__file__).resolve().parent / "collected_coordinates.txt"
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        mode_name = self._current_wmps_guide_mode_name(self.laser_measurement_wmps_mode)
        origin = np.asarray(self.distance_constraint_origin_mm, dtype=float).reshape(3)
        source_text = f" | {self.laser_measurement_wmps_source}" if self.laser_measurement_wmps_source else ""
        with txt_path.open("a", encoding="utf-8") as f:
            f.write(
                f"{timestamp} | 测距约束 | {mode_name} | "
                f"distance={distance_mm:.4f}mm | "
                f"origin=({origin[0]:.6f},{origin[1]:.6f},{origin[2]:.6f}) | "
                f"raw=({raw_xyz[0]:.4f},{raw_xyz[1]:.4f},{raw_xyz[2]:.4f}) | "
                f"constrained=({constrained_xyz[0]:.4f},{constrained_xyz[1]:.4f},{constrained_xyz[2]:.4f}) | "
                f"femtosecond_frame=({local_xyz[0]:.4f},{local_xyz[1]:.4f},{local_xyz[2]:.4f}) | "
                f"raw_radius={raw_radius_mm:.4f}mm | correction={correction_mm:.4f}mm{source_text}\n"
            )

    def _publish_coordinate_solution(self, local_xyz, constrained_xyz, raw_xyz, distance_mm):
        local_xyz = np.asarray(local_xyz, dtype=float).reshape(3)
        constrained_xyz = np.asarray(constrained_xyz, dtype=float).reshape(3)
        raw_xyz = np.asarray(raw_xyz, dtype=float).reshape(3)
        x, y, z = local_xyz
        self.coordinate_result.setText(f"X={x:.4f}, Y={y:.4f}, Z={z:.4f}")
        self.coordinate_result.setToolTip(
            "全向飞秒测距仪原点坐标系（轴向沿用wMPS）\n"
            f"局部坐标: ({x:.6f}, {y:.6f}, {z:.6f}) mm\n"
            f"wMPS四靶标角锥: ({raw_xyz[0]:.6f}, {raw_xyz[1]:.6f}, {raw_xyz[2]:.6f}) mm\n"
            f"融合全局坐标: ({constrained_xyz[0]:.6f}, {constrained_xyz[1]:.6f}, {constrained_xyz[2]:.6f}) mm\n"
            f"飞秒距离: {distance_mm:.6f} mm"
        )

        try:
            active_positions = self.wmps_controller.get_transmitter_origins_global_mm()
        except Exception as exc:
            print(f"[3D显示] 当前wMPS发射站坐标读取失败: {exc}")
            active_positions = {}
        if self.last_wmps_transmitter_ids:
            participating_positions = {
                transmitter_id: active_positions[transmitter_id]
                for transmitter_id in self.last_wmps_transmitter_ids
                if transmitter_id in active_positions
            }
            if participating_positions:
                active_positions = participating_positions
        if active_positions:
            self._set_wmps_transmitter_positions(
                active_positions,
                "当前实时解算所加载外参",
            )
        else:
            self._refresh_coordinate_3d_view()

    def _apply_laser_distance_constraint(self, distance_mm, wmps_xyz=None):
        if wmps_xyz is None:
            wmps_xyz = self.laser_measurement_wmps_xyz
        if wmps_xyz is None or self.laser_measurement_wmps_mode != GUIDE_MODE_MULTI:
            print("[测距约束] 本次测距没有绑定四靶标角锥坐标，跳过融合解算")
            return None

        try:
            raw_xyz = np.asarray(wmps_xyz, dtype=float).reshape(3)
            constrained_xyz = np.asarray(
                constrain_point_to_distance_sphere(
                    raw_xyz,
                    distance_mm,
                    self.distance_constraint_origin_mm,
                ),
                dtype=float,
            ).reshape(3)
            local_xyz = np.asarray(
                wmps_global_to_femtosecond(
                    constrained_xyz,
                    self.distance_constraint_origin_mm,
                ),
                dtype=float,
            ).reshape(3)
        except Exception as exc:
            print(f"[测距约束] 距离约束失败: {exc}")
            return None

        origin = np.asarray(self.distance_constraint_origin_mm, dtype=float).reshape(3)
        raw_radius_mm = float(np.linalg.norm(raw_xyz - origin))
        constrained_radius_mm = float(np.linalg.norm(constrained_xyz - origin))
        correction_mm = float(np.linalg.norm(constrained_xyz - raw_xyz))
        constrained_tuple = tuple(float(v) for v in constrained_xyz)
        local_tuple = tuple(float(v) for v in local_xyz)

        self.last_distance_constrained_wmps_xyz = constrained_tuple
        self.last_distance_constrained_femtosecond_xyz = local_tuple
        self.last_four_marker_corner_wmps_xyz = tuple(float(v) for v in raw_xyz)
        self.last_solution_distance_mm = float(distance_mm)
        self.last_collected_wmps_xyz = constrained_tuple
        self.last_collected_wmps_mode = GUIDE_MODE_MULTI
        self.wmps_receiver.set_collected_coords(*constrained_tuple)
        self._append_laser_distance_constraint_log(
            raw_xyz,
            constrained_xyz,
            local_xyz,
            distance_mm,
            raw_radius_mm,
            correction_mm,
        )
        self._publish_coordinate_solution(
            local_xyz,
            constrained_xyz,
            raw_xyz,
            distance_mm,
        )

        print(
            f"[测距约束] 已生成四靶标+飞秒融合坐标: "
            f"raw=({raw_xyz[0]:.4f}, {raw_xyz[1]:.4f}, {raw_xyz[2]:.4f}) | "
            f"global=({constrained_xyz[0]:.4f}, {constrained_xyz[1]:.4f}, {constrained_xyz[2]:.4f}) | "
            f"femtosecond_frame=({local_xyz[0]:.4f}, {local_xyz[1]:.4f}, {local_xyz[2]:.4f}) | "
            f"distance={distance_mm:.4f} mm | radius={constrained_radius_mm:.4f} mm | "
            f"raw_radius={raw_radius_mm:.4f} mm | correction={correction_mm:.4f} mm"
        )
        return local_tuple

    def solve_coordinate_once(self):
        """Publish one coordinate from the paired four-marker/range snapshot."""
        distance_mm = self.last_final_femtosecond_distance_mm
        if (
            distance_mm is None
            or not np.isfinite(distance_mm)
            or distance_mm <= 0.0
        ):
            message = "无有效的最终飞秒测距值"
            self.coordinate_result.setText(message)
            print(f"[坐标解算] {message}，请先完成一次测距")
            return None

        coords = self.laser_measurement_wmps_xyz
        if coords is None or self.laser_measurement_wmps_mode != GUIDE_MODE_MULTI:
            message = "本次测距未绑定四靶标角锥快照"
            self.coordinate_result.setText(message)
            print(f"[坐标解算] {message}，请保持靶标有效并重新测距")
            return None

        if (
            self.last_distance_constrained_femtosecond_xyz is not None
            and self.last_distance_constrained_wmps_xyz is not None
            and self.last_four_marker_corner_wmps_xyz is not None
            and self.last_solution_distance_mm is not None
            and abs(float(self.last_solution_distance_mm) - float(distance_mm)) < 1e-9
            and np.allclose(
                np.asarray(self.last_four_marker_corner_wmps_xyz, dtype=float),
                np.asarray(coords, dtype=float),
                rtol=0.0,
                atol=1e-9,
            )
        ):
            result = tuple(self.last_distance_constrained_femtosecond_xyz)
            self._publish_coordinate_solution(
                result,
                self.last_distance_constrained_wmps_xyz,
                self.last_four_marker_corner_wmps_xyz,
                distance_mm,
            )
            print(
                "[坐标解算] 已重新显示本次四靶标+飞秒融合结果: "
                f"X={result[0]:.4f}, Y={result[1]:.4f}, Z={result[2]:.4f} mm"
            )
            return result

        result = self._apply_laser_distance_constraint(distance_mm, coords)
        if result is None:
            self.coordinate_result.setText("坐标融合解算失败")
            return None

        print(
            "[坐标解算] 已使用本次测距绑定的四靶标快照输出一次坐标: "
            f"X={result[0]:.4f}, Y={result[1]:.4f}, Z={result[2]:.4f} mm"
        )
        return result

    def _do_start_measurement(self):
        if self.laser_cmd is None:
            print("[Laser] serial is not connected")
            return False

        self.last_final_femtosecond_distance_mm = None
        self.last_distance_constrained_wmps_xyz = None
        self.last_distance_constrained_femtosecond_xyz = None
        self.last_solution_distance_mm = None
        self.coordinate_result.clear()
        self._refresh_coordinate_3d_view()

        lp = self.laser_processor

        rude_distance = self._resolve_laser_rude_distance()
        if rude_distance is None:
            return False
        lp.rude_distance = rude_distance

        self.laser_auto_stop_timer.stop()
        self.laser_lpp_timeout_timer.stop()
        self.laser_waiting_for_lpp = False

        lp.raw_ch1.clear()
        lp.raw_ch2.clear()
        lp.all_group_distances.clear()
        lp.reset_cumulative()
        lp.reset_lpp_state()
        lp.group_index = 0
        lp.current_ng = None
        lp.current_fr1 = None
        lp.measurement_active = True

        self.laser_cmd.start_measurement()
        self.laser_auto_stop_timer.start(self.laser_auto_measure_ms)
        return True

    def stop_measurement(self):
        self._stop_measurement_and_query_lpp()

    def _stop_measurement_and_query_lpp(self):
        if self.laser_cmd is None:
            print("[Laser] serial is not connected")
            self._reset_laser_measurement_ui()
            return
        if self.laser_waiting_for_lpp:
            return

        self.laser_auto_stop_timer.stop()
        self.laser_processor.set_measurement_active(True)
        self.laser_processor.prepare_lpp_finalize(expected_lpp_commands=1)
        self.laser_waiting_for_lpp = True
        self.fs_start_measurement.setText("\u7b49\u5f85LPP")
        self.fs_start_measurement.setEnabled(False)

        self.laser_cmd.stop_measurement()
        QTimer.singleShot(self.laser_lpp_query_delay_ms, self._query_lpp_after_stop)

    def _query_lpp_after_stop(self):
        if not self.laser_waiting_for_lpp or self.laser_cmd is None:
            return
        self.laser_cmd.query_lpp()
        QTimer.singleShot(self.laser_lpp_retry_delay_ms, self._retry_query_lpp_after_stop)
        self.laser_lpp_timeout_timer.start(self.laser_lpp_timeout_ms)

    def _retry_query_lpp_after_stop(self):
        if not self.laser_waiting_for_lpp or self.laser_cmd is None:
            return
        self.laser_cmd.query_lpp()

    def on_laser_lpp_ready(self, lpp_mm):
        self.last_lpp_mm = lpp_mm
        self._set_lpp_value_display(lpp_mm)

        if self.laser_waiting_for_lpp:
            print(f"[Laser] LPP received: {lpp_mm:.6f} mm")
            self._reset_laser_measurement_ui()
            return

        if self.manual_lpp_query_active:
            self.manual_lpp_query_active = False
            self.manual_lpp_timeout_timer.stop()
            print(f"[Laser] manual LPP: {lpp_mm:.6f} mm")

    def query_lpp_value(self):
        if self.laser_cmd is None:
            print("[Laser] serial is not connected")
            return
        if self.laser_waiting_for_lpp:
            print("[Laser] measurement is waiting for LPP, skip manual LPP query")
            return

        self.manual_lpp_query_active = True
        self.lpp_value_display.setText("LPP: reading...")
        self.laser_cmd.query_lpp()
        QTimer.singleShot(self.laser_lpp_retry_delay_ms, self._retry_manual_lpp_query)
        self.manual_lpp_timeout_timer.start(self.laser_lpp_timeout_ms)

    def _retry_manual_lpp_query(self):
        if not self.manual_lpp_query_active or self.laser_cmd is None:
            return
        self.laser_cmd.query_lpp()

    def _manual_lpp_timeout(self):
        if not self.manual_lpp_query_active:
            return
        self.manual_lpp_query_active = False
        self.lpp_value_display.setText("LPP: timeout")
        print("[Laser] manual LPP timeout")

    def _set_lpp_value_display(self, lpp_mm):
        text = f"LPP: {lpp_mm:.3f} mm"
        if hasattr(self, "lpp_value_display"):
            self.lpp_value_display.setText(text)

    def _finalize_laser_after_lpp_timeout(self):
        if not self.laser_waiting_for_lpp:
            return
        print("[Laser] LPP timeout, finalize with fallback lpp")
        finalized = self.laser_processor.finalize_pending_lpp_measurement()
        if not finalized:
            self.laser_processor.finalize_results()
        self._reset_laser_measurement_ui()

    def _reset_laser_measurement_ui(self):
        self.laser_auto_stop_timer.stop()
        self.laser_lpp_timeout_timer.stop()
        self.laser_waiting_for_lpp = False
        self.fs_start_measurement.setEnabled(True)
        self.fs_start_measurement.setText("\u6d4b\u91cf")

    def on_laser_rolling_result(self, result):
        """累积平均实时结果"""
        tag = " [剔]" if result["filtered"] else ""
        print(
            f"[累积平均] 均值: {result['cumulative_mean_mm']:.3f} mm | "
            f"std: {result['cumulative_std_mm']:.3f} | "
            f"有效数: {result['valid_count']} | "
            f"累计异常: {result['outlier_total']}"
            f"{tag}"
        )
        self.last_femtosecond_distance_mm = float(result["cumulative_mean_mm"])
        self.measurement_result.setText(f'{result["cumulative_mean_mm"]:.3f}')

    def _current_wmps_guide_mode(self):
        text = self.guide_mode_combo.currentText().strip()
        if text == GUIDE_MODE_TEXT[GUIDE_MODE_SINGLE]:
            return GUIDE_MODE_SINGLE
        if text == GUIDE_MODE_TEXT[GUIDE_MODE_MULTI]:
            return GUIDE_MODE_MULTI
        return None

    def _current_wmps_guide_mode_name(self, mode=None):
        if mode is None:
            mode = self._current_wmps_guide_mode()
        return GUIDE_MODE_TEXT.get(mode, "未选择模式")

    def _on_wmps_guide_mode_changed(self, _text):
        self.last_collected_wmps_xyz = None
        self.last_collected_wmps_mode = None
        self.last_wmps_guidance_source = ""
        self.last_wmps_guidance_diag = None
        if getattr(self, "realtime_guide_enabled", False):
            self.stop_realtime_wmps_guidance()
            print("[实时引导] 引导模式已切换，请重新启动实时引导")

    def _load_wmps_center_solver(self, log_prefix="[多点引导]", suppress_errors=False):
        if self.wmps_center_solver is not None and self.wmps_center_params is not None:
            return True

        solver_dir = Path(__file__).resolve().parent / "靶标标定"
        solver_path = solver_dir / "wmps_center_solver.py"
        params_path = solver_dir / "wmps_center_params.json"
        if not solver_path.exists() or not params_path.exists():
            if not suppress_errors:
                print(f"{log_prefix} 缺少四靶标解算程序或参数: {solver_path}, {params_path}")
            return False

        try:
            spec = importlib.util.spec_from_file_location("wmps_center_solver_runtime", solver_path)
            if spec is None or spec.loader is None:
                raise ImportError(f"cannot load {solver_path}")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.wmps_center_params = module.load_params(params_path)
            self.wmps_center_solver = module
            self.wmps_center_warn_rms_mm = float(getattr(module, "WARN_RIGID_RMS_MM", 1.0))
            if not suppress_errors:
                print(f"{log_prefix} 已加载四靶标标定参数: {params_path}")
            return True
        except Exception as exc:
            self.wmps_center_solver = None
            self.wmps_center_params = None
            if not suppress_errors:
                print(f"{log_prefix} 加载四靶标解算程序失败: {exc}")
            return False

    @staticmethod
    def _wmps_receiver_sort_key(receiver_id):
        try:
            processor_id, channel = str(receiver_id).split("@", 1)
            return 0, int(processor_id), int(channel), str(receiver_id)
        except (TypeError, ValueError):
            return 1, 0, 0, str(receiver_id)

    def _normalize_wmps_xyz(self, value, label, log_prefix="[wMPS]", suppress_errors=False):
        try:
            coords = np.asarray(value, dtype=float).reshape(-1)
        except Exception as exc:
            if not suppress_errors:
                print(f"{log_prefix} wMPS {label}无效: {exc}")
            return None

        if coords.size < 3 or not np.all(np.isfinite(coords[:3])):
            if not suppress_errors:
                print(f"{log_prefix} wMPS {label}无效: {coords}")
            return None
        return float(coords[0]), float(coords[1]), float(coords[2])

    def _get_valid_wmps_samples(self, log_prefix="[wMPS]", suppress_errors=False):
        samples = []
        for receiver_id in sorted(self.wmps_controller.m_processorCoordinate.keys(), key=self._wmps_receiver_sort_key):
            if self.wmps_controller.m_receiverStatus.get(receiver_id, 2) != 1:
                continue
            coords = self._normalize_wmps_xyz(
                self.wmps_controller.m_processorCoordinate[receiver_id],
                f"{receiver_id} 坐标",
                log_prefix,
                suppress_errors,
            )
            if coords is not None:
                samples.append((receiver_id, coords))
        return samples

    def _read_single_point_wmps_xyz(self, log_prefix="[引导]", suppress_errors=False):
        samples = self._get_valid_wmps_samples(log_prefix, suppress_errors)
        if len(samples) == 1:
            receiver_id, coords = samples[0]
            self.last_wmps_guidance_source = f"单点:{receiver_id}"
            self.last_wmps_guidance_diag = None
            if not suppress_errors:
                print(f"{log_prefix} 使用单点接收器: {receiver_id}")
            return coords

        if not suppress_errors:
            if not samples:
                print(f"{log_prefix} 单点引导没有有效 wMPS 坐标")
            else:
                ids = ", ".join(receiver_id for receiver_id, _coords in samples)
                print(f"{log_prefix} 单点引导检测到多个 wMPS 坐标: {ids}，请切到多点引导或只保留一个接收器")
        return None

    def _select_four_marker_samples(self, samples):
        by_processor = {}
        for receiver_id, coords in samples:
            try:
                processor_id, channel = str(receiver_id).split("@", 1)
                processor_id = int(processor_id)
                channel = int(channel)
            except (TypeError, ValueError):
                continue
            by_processor.setdefault(processor_id, {})[channel] = (receiver_id, coords)

        for channel_order in ((0, 1, 2, 3), (1, 2, 3, 4)):
            for processor_id in sorted(by_processor):
                channels = by_processor[processor_id]
                if all(channel in channels for channel in channel_order):
                    return [channels[channel] for channel in channel_order]

        return None

    def _read_multi_point_wmps_xyz(self, log_prefix="[引导]", suppress_errors=False):
        snapshot = self.wmps_controller.get_latest_four_marker_snapshot()
        if snapshot is None:
            if not suppress_errors:
                samples = self._get_valid_wmps_samples(log_prefix, True)
                ids = ", ".join(receiver_id for receiver_id, _coords in samples) or "无"
                print(
                    f"{log_prefix} 四靶标解算需要同一处理器、同一新鲜数据包中的"
                    f"通道 0-3 或 1-4，当前缓存: {ids}"
                )
            return None

        if not self._load_wmps_center_solver(log_prefix, suppress_errors):
            return None

        marker_ids = list(snapshot["receiver_ids"])
        markers = np.asarray(snapshot["coordinates"], dtype=float)
        try:
            center, diag = self.wmps_center_solver.solve_center_from_four_markers(
                markers,
                self.wmps_center_params,
            )
        except Exception as exc:
            if not suppress_errors:
                print(f"{log_prefix} 四接收器解算角锥坐标失败: {exc}")
            return None

        center = np.asarray(center, dtype=float).reshape(-1)
        if center.size < 3 or not np.all(np.isfinite(center[:3])):
            if not suppress_errors:
                print(f"{log_prefix} 四接收器解算结果无效: {center}")
            return None

        rigid_rms = float(diag.get("rigid_marker_rms_mm", float("inf")))
        if not np.isfinite(rigid_rms) or rigid_rms > self.wmps_center_warn_rms_mm:
            if not suppress_errors:
                print(
                    f"{log_prefix} 拒绝四靶标角锥坐标: "
                    f"rigid_rms={rigid_rms:.4f} mm，阈值="
                    f"{self.wmps_center_warn_rms_mm:.4f} mm；请检查靶标顺序或数据质量"
                )
            return None

        self.last_wmps_guidance_source = (
            "四靶标同包:"
            + " -> ".join(marker_ids)
            + f" | age={snapshot['age_seconds']:.3f}s"
        )
        self.last_wmps_guidance_diag = diag
        self.last_wmps_transmitter_ids = tuple(snapshot.get("transmitter_ids", ()))
        if not suppress_errors:
            print(
                f"{log_prefix} 多点解算角锥坐标: "
                f"X={center[0]:.4f}, Y={center[1]:.4f}, Z={center[2]:.4f} | "
                f"rigid_rms={rigid_rms:.4f} mm | "
                f"数据龄={snapshot['age_seconds']:.3f} s | "
                f"顺序={', '.join(marker_ids)}"
            )
        return float(center[0]), float(center[1]), float(center[2])

    def _read_current_guidance_xyz(self, log_prefix="[引导]", suppress_errors=False):
        self.last_wmps_guidance_source = ""
        self.last_wmps_guidance_diag = None
        mode = self._current_wmps_guide_mode()
        if mode is None:
            if not suppress_errors:
                print(f"{log_prefix} 请先选择单点引导或多点引导模式")
            return None
        if mode == GUIDE_MODE_SINGLE:
            return self._read_single_point_wmps_xyz(log_prefix, suppress_errors)
        if mode == GUIDE_MODE_MULTI:
            return self._read_multi_point_wmps_xyz(log_prefix, suppress_errors)
        return None

    def _read_latest_wmps_xyz(self):
        return self._read_current_guidance_xyz()

    def _collect_current_value(self):
        """采集按钮：截取当前引导模式的 wMPS XYZ，保存并显示"""
        mode = self._current_wmps_guide_mode()
        coords = self._read_current_guidance_xyz("[采集]")
        if coords is None:
            print("[采集] wMPS坐标尚不可用，无法保存")
            return None

        x, y, z = coords
        mode_name = self._current_wmps_guide_mode_name(mode)
        self.last_collected_wmps_xyz = coords
        self.last_collected_wmps_mode = mode

        diag_text = ""
        if self.last_wmps_guidance_diag is not None:
            rigid_rms = self.last_wmps_guidance_diag.get("rigid_marker_rms_mm")
            if rigid_rms is not None:
                diag_text = f" | rigid_rms={float(rigid_rms):.4f}mm"
        source_text = f" | {self.last_wmps_guidance_source}" if self.last_wmps_guidance_source else ""

        txt_path = Path(__file__).resolve().parent / "collected_coordinates.txt"
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        with txt_path.open("a", encoding="utf-8") as f:
            f.write(f"{timestamp} | {mode_name} | X={x:.4f}  Y={y:.4f}  Z={z:.4f}{source_text}{diag_text}\n")
        print(f"[采集] 已保存{mode_name}: X={x:.4f}, Y={y:.4f}, Z={z:.4f}{diag_text} -> {txt_path}")

        self.wmps_receiver.set_collected_coords(x, y, z)
        return coords

    def _convert_wmps_coords_to_motor_angles(self, coords, log_prefix="[引导]"):
        x, y, z = coords
        try:
            from manual_xyz_to_pitch_yaw import xyz_to_pitch_yaw

            result = xyz_to_pitch_yaw(x, y, z)
            pitch_cmd = float(result["pitch_cmd"])
            yaw_cmd = float(result["yaw_cmd"])
        except Exception as exc:
            print(f"{log_prefix} 坐标转换为 pitch/yaw 失败: {exc}")
            return None

        if not np.all(np.isfinite([pitch_cmd, yaw_cmd])):
            print(f"{log_prefix} 转换结果无效: pitch={pitch_cmd}, yaw={yaw_cmd}")
            return None

        return pitch_cmd, yaw_cmd, result

    def toggle_realtime_wmps_guidance(self):
        if self.realtime_guide_enabled:
            self.stop_realtime_wmps_guidance()
            return

        mode = self._current_wmps_guide_mode()
        if mode is None:
            print("[实时引导] 请先选择单点引导或多点引导模式")
            return

        if self.controller is None:
            print("[实时引导] 控制器未连接，无法启动闭环")
            return

        coords = self._read_current_guidance_xyz("[实时引导]")
        if coords is None:
            print("[实时引导] wMPS坐标尚不可用，无法启动闭环")
            return

        self.realtime_guide_enabled = True
        self._last_realtime_guide_cmd = None
        self._realtime_guide_log_time = 0.0
        self.realtime_guide_button.setText("停止引导")
        self.realtime_guide_timer.start()
        print(f"[实时引导] 已启动 {self._current_wmps_guide_mode_name(mode)} 实时闭环")
        self._run_realtime_wmps_guidance()

    def stop_realtime_wmps_guidance(self):
        if not self.realtime_guide_enabled:
            return
        self.realtime_guide_enabled = False
        self.realtime_guide_timer.stop()
        self._last_realtime_guide_cmd = None
        self.pending_pitch = None
        self.pending_yaw = None
        self.realtime_guide_button.setText("实时引导")
        print("[实时引导] 已停止")

    def _run_realtime_wmps_guidance(self):
        if not self.realtime_guide_enabled:
            return

        mode = self._current_wmps_guide_mode()
        if mode is None:
            print("[实时引导] 引导模式未设置，停止闭环")
            self.stop_realtime_wmps_guidance()
            return

        if self.controller is None:
            print("[实时引导] 控制器断开，停止闭环")
            self.stop_realtime_wmps_guidance()
            return

        now = time.time()
        coords = self._read_current_guidance_xyz("[实时引导]", suppress_errors=True)
        if coords is None:
            if now - self._realtime_guide_log_time > 2.0:
                mode_name = self._current_wmps_guide_mode_name(mode)
                print(f"[实时引导] 等待有效 {mode_name} wMPS 坐标")
                self._realtime_guide_log_time = now
            return

        converted = self._convert_wmps_coords_to_motor_angles(coords, "[实时引导]")
        if converted is None:
            return

        pitch_cmd, yaw_cmd, result = converted
        last = self._last_realtime_guide_cmd
        if last is not None:
            if max(abs(pitch_cmd - last[0]), abs(yaw_cmd - last[1])) < self.realtime_guide_deadband_deg:
                return

        self.pitch_angle.setValue(pitch_cmd)
        self.yaw_angle.setValue(yaw_cmd)
        pitch_to_send = float(self.pitch_angle.value())
        yaw_to_send = float(self.yaw_angle.value())

        self.pending_pitch = pitch_to_send
        self.pending_yaw = yaw_to_send
        self._last_realtime_guide_cmd = (pitch_to_send, yaw_to_send)
        self._sendMotorIfPending()

        if now - self._realtime_guide_log_time > 1.0:
            x, y, z = coords
            mode_name = self._current_wmps_guide_mode_name(mode)
            source_text = f" | {self.last_wmps_guidance_source}" if self.last_wmps_guidance_source else ""
            diag_text = ""
            if self.last_wmps_guidance_diag is not None:
                rigid_rms = self.last_wmps_guidance_diag.get("rigid_marker_rms_mm")
                if rigid_rms is not None:
                    diag_text = f" | rigid_rms={float(rigid_rms):.3f} mm"
            print(
                f"[实时引导] {mode_name} -> 电机: "
                f"X={x:.4f}, Y={y:.4f}, Z={z:.4f} | "
                f"pitch={pitch_to_send:.6f}, yaw={yaw_to_send:.6f} | "
                f"model_err={result.get('aim_error_cmd_model_mm', 0.0):.3f} mm"
                f"{diag_text}{source_text}"
            )
            self._realtime_guide_log_time = now

    def _guide_to_collected_value(self):
        """引导按钮：把当前模式的 wMPS 坐标转换为转台角度并发送给电机。"""
        mode = self._current_wmps_guide_mode()
        if mode is None:
            print("[引导] 请先选择单点引导或多点引导模式")
            return

        coords = self.last_collected_wmps_xyz
        if coords is None or self.last_collected_wmps_mode != mode:
            coords = self._collect_current_value()
            if coords is None:
                return

        converted = self._convert_wmps_coords_to_motor_angles(coords, "[引导]")
        if converted is None:
            return

        pitch_cmd, yaw_cmd, _result = converted
        x, y, z = coords
        self.pitch_angle.setValue(pitch_cmd)
        self.yaw_angle.setValue(yaw_cmd)
        pitch_to_send = float(self.pitch_angle.value())
        yaw_to_send = float(self.yaw_angle.value())

        if abs(pitch_to_send - pitch_cmd) > 1e-9 or abs(yaw_to_send - yaw_cmd) > 1e-9:
            print(
                "[引导] 转换角度超出输入框范围，已按界面限制截断: "
                f"pitch={pitch_to_send:.6f}, yaw={yaw_to_send:.6f}"
            )

        mode_name = self._current_wmps_guide_mode_name(mode)
        print(
            f"[引导] {mode_name} wMPS坐标 -> 电机角度: "
            f"X={x:.4f}, Y={y:.4f}, Z={z:.4f} | "
            f"pitch={pitch_to_send:.6f}, yaw={yaw_to_send:.6f}"
        )

        if self.controller is None:
            self.pending_pitch = None
            self.pending_yaw = None
            print("[引导] 控制器未连接，已更新角度输入框，但未发送到电机")
            return

        try:
            self.pending_pitch = pitch_to_send
            self.pending_yaw = yaw_to_send
            sent = self._sendMotorIfPending()
            if sent:
                print("[引导] 已发送到电机")
            else:
                print("[引导] 电机命令未发送成功，请检查控制器连接")
        except Exception as exc:
            print(f"[引导] 发送电机命令失败: {exc}")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
