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
- 单个“测试PSD和电机频率”诊断按钮。

初始化并使能双轴后，点击“测试PSD和电机频率（约8秒）”可测纯PSD极限、设定频率并行、双轴电机极限和并行极限。测速与跟踪启动都会自动尝试取得新的双轴反馈。下位机转发的电机反馈仍会更新界面角度。

V10的PSD控制发生在下位机位置模式中。当前下位机目标为PSD采样1 kHz、电机链路每轴290 Hz；两轴各有新反馈才触发一次控制计算。每轮目标只用最新电机反馈位置与最新PSD修正量计算。上位机不承担实时控制时钟。上位机“速度模式”按钮仅用于人工调试，不是开启PSD跟踪的前置步骤。

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
