# laser_serial_controller.py

class LaserSerialController:
    """
    测距仪指令控制层
    （从树莓派程序迁移过来）
    """

    def __init__(self, serial_worker):
        """
        serial_worker: LaserSerialWorker 实例
        """
        self.ser = serial_worker

    # ========================
    # 基础写接口
    # ========================
    def _send(self, cmd: str):
        self.ser.write(cmd)

    # ========================
    # 测距仪指令（与你原来一致）
    # ========================

    def enter_control_mode(self):
        """CPU1"""
        self._send("CPU1")

    def laser_on(self, power=5):
        """LON"""
        self._send(f"LON, {power}")

    def laser_off(self, power=5):
        """LFF"""
        self._send(f"LFF, {power}")

    def set_sfc(self, value: int):
        """SFC"""
        self._send(f"SFC, {value}")

    def set_lfc(self, value: int):
        """LFC"""
        self._send(f"LFC, {value}")

    def set_target_num(self, num: int):
        """TAR"""
        self._send(f"TAR, {num}")

    def set_threshold_ref(self, value: int):
        """TH1"""
        self._send(f"TH1, {value}")

    def set_threshold_sig(self, value: int):
        """TH2"""
        self._send(f"TH2, {value}")

    def enter_adjust_mode(self):
        """AJT"""
        self._send("AJT")

    def set_sig(self, value: int):
        """SIG"""
        self._send(f"SIG, {value}")

    def query_temperature(self):
        """TEM"""
        self._send("TEM")

    def start_measurement(self):
        """开始测距（MEA,1）"""
        self._send("MEA,1")

    def stop_measurement(self):
        """停止测距（1）"""
        self._send("1")

    def query_lpp(self):
        """LPP"""
        self._send("LPP")
