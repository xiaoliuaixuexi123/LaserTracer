"""Static interface check for the packaged V10 upper machine."""

import ast
from pathlib import Path


HERE = Path(__file__).resolve().parent


def class_methods(path, class_name):
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return {
                item.name
                for item in node.body
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
            }
    raise AssertionError(f"class not found: {class_name}")


main_path = HERE / "main.py"
network_path = HERE / "pyside_to_raspberrypi.py"
main_methods = class_methods(main_path, "MainWindow")
network_methods = class_methods(network_path, "RobotNetworkController")

assert {
    "runMaxRateTest",
    "save_current_position",
    "sendMotorMessage",
    "startPsdTracking",
    "stopPsdTracking",
    "motorPositionMode",
    "velocityPositionMode",
    "setZeroPosition",
    "enableMotor",
    "disableMotor",
}.issubset(main_methods)

assert {
    "getMotorStatus",
    "getAllMotorsStatus",
    "moveToPosition",
    "positionControl",
    "velocityControl",
    "setZero",
    "startTracking",
    "stopTracking",
    "benchmarkTrackingRates",
    "testMotorPositionLoop",
    "testMotorPositionLoopOnce",
    "stopMotorPositionLoopTest",
}.issubset(network_methods)

source = main_path.read_text(encoding="utf-8-sig")
assert "query_motor_position_button" not in source
assert "测试PSD和电机频率（约8秒）" in source
assert "核查CANFD上传开关（约14秒）" not in source
assert "链路采样：停（点此开）" not in source
assert "电机位置环辨识（约45秒）" not in source
assert "单次阶跃+自动反馈（约45秒）" not in source
assert "V10位置跟踪版" in source
assert source.count("pole_pairs=21") >= 2
assert source.count("model='HO7213'") >= 2
assert "Kp=0x06, Kd=0xF0, velocity=2, currency=3" in source
assert "Kp=0x10, Kd=0xA0, velocity=2, currency=3" in source
assert "'motor1', Kp=0x1E, Kd=0xD2, velocity=2, currency=2" in source
assert "'motor2', Kp=0x1C, Kd=0x80, velocity=2, currency=2" in source

network_source = network_path.read_text(encoding="utf-8-sig")
assert "pole_pairs=21" in network_source
assert "'pole_pairs': pole_pairs" in network_source
assert "model='HO7213'" in network_source
assert "'model': model" in network_source

print("V10 upper-machine interface self-test: PASS")
