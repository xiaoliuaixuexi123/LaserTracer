# V10位置跟踪版上位机

本目录与同级V10下位机配套使用：

```text
tracking_test_versions/10_position_multirate_fast/raspberrypi_to_pyside.py
tracking_test_versions/10_position_multirate_fast/upper_machine_v10/main.py
```

## 保留功能

- 添加、初始化、使能和失能双轴电机；
- 位置模式、手动位置设定和点动；
- 手动速度模式调试；
- 设置零点；
- `start_tracking`和`stop_tracking`；
- 飞秒测距、wMPS和坐标解算；
- 记录当前位置；
- 独立的“查询当前位置”按钮。

“查询当前位置”会分别向`motor1`和`motor2`发送真实`get_status`，下位机反馈后更新界面中的实际俯仰角和实际偏航角。“记录当前位置”仍只负责把界面角度写入文件，两者互不替代。

V10的PSD控制发生在下位机位置模式中。上位机“速度模式”按钮仅用于人工调试，不是开启PSD跟踪的前置步骤。

## 安装依赖

```powershell
python -m pip install -r requirements.txt
```

## 启动

```powershell
python self_test.py
python main.py
```

下位机启动：

```bash
cd /home/wheeltec/Documents/PSD/10_position_multirate_fast
sudo /usr/bin/python3 raspberrypi_to_pyside.py
```
