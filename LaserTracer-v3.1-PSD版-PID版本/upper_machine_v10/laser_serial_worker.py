# laser_serial_worker.py
import serial
import threading
import time
from PySide6.QtCore import QObject, Signal


class LaserSerialWorker(QObject):
    """
    测距仪串口通信线程
    - 负责：串口读 / 写
    - 不负责：协议解析、物理计算
    """

    raw_line = Signal(str)     # 接收到的一行原始数据
    error = Signal(str)        # 串口错误信息
    connected = Signal(str)   # 成功连接
    disconnected = Signal()   # 断开连接

    def __init__(self, port: str, baud: int = 921600):
        super().__init__()
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self._thread = None

    # ========================
    # 串口控制
    # ========================
    def start(self):
        """打开串口并启动接收线程"""
        if self.running:
            return

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1
            )
            self.running = True
            self._thread = threading.Thread(
                target=self._read_loop,
                daemon=True
            )
            self._thread.start()

            self.connected.emit(self.port)
            print(f"[LaserSerial] 已连接串口: {self.port}")

        except Exception as e:
            self.error.emit(str(e))
            print(f"[LaserSerial] 串口连接失败: {e}")

    def stop(self):
        """停止接收并关闭串口"""
        self.running = False

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        self.disconnected.emit()
        print("[LaserSerial] 串口已断开")

    # ========================
    # 串口写指令
    # ========================
    def write(self, cmd: str):
        """
        向测距仪发送一条指令
        自动补 CRLF
        """
        if not self.ser or not self.ser.is_open:
            self.error.emit("串口未连接，无法发送指令")
            return

        if not cmd.endswith("\r\n"):
            cmd += "\r\n"

        try:
            self.ser.write(cmd.encode("utf-8"))
            # print(f"[LaserSerial TX] {cmd.strip()}")
        except Exception as e:
            self.error.emit(str(e))

    # ========================
    # 串口读线程
    # ========================
    def _read_loop(self):
        buffer = ""

        while self.running:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(
                        self.ser.in_waiting
                    ).decode("utf-8", errors="ignore")

                    buffer += data

                    # 按行分割（兼容 \n / \r）
                    while "\n" in buffer or "\r" in buffer:
                        if "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                        else:
                            line, buffer = buffer.split("\r", 1)

                        line = line.strip()
                        if line:
                            # print(f"[LaserSerial RX] {line}")
                            self.raw_line.emit(line)

                else:
                    time.sleep(0.002)

            except Exception as e:
                self.error.emit(str(e))
                time.sleep(0.05)
