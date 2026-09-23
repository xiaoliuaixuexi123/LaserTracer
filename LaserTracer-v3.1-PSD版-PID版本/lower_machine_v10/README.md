# 10号：PSD 1 kHz采样、双轴反馈驱动位置跟踪

## 频率选择依据

v19线程采集实测：PSD单独1852.1 Hz，设定并行时PSD 268.1 Hz、
双轴反馈229.2 Hz；电机单独回帧455.8 Hz，极限并行时PSD 274.2 Hz、
双轴回帧415.1 Hz。v20曾改用独立进程；v22按要求切回同一进程内的PSD采集线程。
290 Hz仍是电机链路设定，跟踪启动要求双轴反馈至少250 Hz；历史线程测速
低于这个门槛，切回后须先用实机的设定并行测速确认实际频率。

离线核查：

```text
python -m unittest discover -s lower_machine_v10 -p test_canfd_upload_diagnostics.py -v
python -m unittest discover -s lower_machine_v10 -p test_feedback_clock.py -v
python -m unittest discover -s lower_machine_v10 -p test_psd_acquisition_process.py -v
python -m unittest discover -s lower_machine_v10 -p test_psd_signal_reacquisition.py -v
python lower_machine_v10/self_test.py
```

第10版基于08号异步位置模式，解决“远区速度提高后中心持续跳动”的问题。它仍保持电机位置模式，不使用速度模式。

## 控制架构

1. AD7606在下位机主进程的采集线程中以1 kHz为目标，只向控制线程发布带时间戳的最新PSD样本。
2. 电机发送线程目标每轴290 Hz；无新目标时重发上一组位置目标，以取得连续命令应答。
3. 电机接收线程记录两轴反馈。只有两轴各收到一个新反馈后，控制线程才运行一轮；实际控制频率跟随较慢的电机反馈，不按固定定时器空转。
4. 控制线程取最新PSD样本；PSD没有新样本时不生成新运动目标。状态中分别记录反馈对频率、实际控制更新频率及PSD采样频率。
5. 每一轮以**最新双轴实测位置 + 当前PSD修正增量**生成目标；旧目标即使仍在发送队列，也不会累积到下一轮。三个分区频率上限均为290 Hz，增量按真实命令间隔相对200 Hz基准缩放。
6. 为保留原250 Hz版本的近似时间常数，PSD滤波系数改为0.1572，锁定、释放和反向确认的样本数按控制频率比例调整。
7. 跟踪启动时自动重发已下发的双轴位置目标，取得新反馈并等待链路测速窗口；最近1秒每轴实测反馈率须达到250 Hz。单个丢帧只跳过对应周期；双轴成对反馈连续中断0.5秒才报错停止。
8. 预测和接近阻尼在当前配置中均关闭；电机继续保持位置模式。

## 关键参数

- PSD采样目标：1000 Hz；双轴反馈触发控制目标：290 Hz；电机链路目标：每轴290 Hz；跟踪启动最低实测反馈：250 Hz/轴
- 全局单次计算修正上限：0.030°；连续290 Hz命令经时间缩放后约为0.021°
- 远区动态领先：0.010～0.040°（到12倍死区达到上限）
- 远区最大命令频率：290 Hz
- 中区动态领先：0.003～0.012°
- 中区最大命令频率：290 Hz
- 精细区目标领先：0.001°
- 精细区最大命令频率：290 Hz
- 增量步长物理增益基准：200 Hz
- 反向制动：向量反向连续4次确认后25 ms
- 预测前馈、接近阻尼：当前关闭
- SUM有效范围：0.2～4.5 V
- HO7213：CANFD 21极对；上位机初始化参数为电机1 Kp/Kd=6/240、电机2=16/160、电流上限3 A。上位机“位置模式”按钮会另设电机1为30/210、电机2为28/128、电流上限2 A；部署前应以实机整定值核对这两组参数。

静止振荡排查按仓库根目录 `HO7213_静止振荡测试.md` 执行；必须先确认电机内部位置环稳定，再调整PSD外环。

## 下位机运行

```bash
cd /home/wheeltec/Documents/PSD/10_position_multirate_fast
/usr/bin/python3 self_test.py
sudo /usr/bin/python3 raspberrypi_to_pyside.py
```

启动后应看到：

```text
[PSD分区高速] 已启动 | 采样目标=1000.0Hz 控制目标=290.0Hz | 控制时钟=motor_feedback
```

测试顺序：初始化并使能双轴后点击上位机“测试PSD和电机频率”；它会重发双轴已下发的位置目标、等待新反馈，再测纯PSD极限、设定频率并行、电机极限和并行极限。跟踪启动也会自动启动链路采样；先静止对中20秒，再缓慢移出，最后进行手持连续移动。目标频率不等于实测频率，以测速结果和跟踪状态中的实测值为准。

## 1 kHz / 290 Hz实机验收

1. 部署本目录的 `raspberrypi_to_pyside.py`、`tracking_service.py`、`psd_acquisition_process.py`、`psd_tracker.py`、`psd_calibration.json` 等文件并重启服务，确认版本为 `2026-09-23-feedback-clock-1k290-thread-v22`，且启动日志显示 `采样后端=thread`；必须同步复制JSON，否则仍会启动独立PSD进程。
2. 初始化、使能双轴后点击唯一的频率测试按钮，查看 `configured` 阶段是否实测PSD至少950 Hz、双轴回帧至少250 Hz。这一阶段比极限测速更能判断设定频率是否同时可用。
3. 跟踪状态中检查 `measured_sample_rate_hz`、最近1秒的 `recent_feedback_pair_rate_hz` 与 `recent_control_rate_hz`，以及 `control_rate_below_minimum`、`psd_stale_cycles` 和 `control_deadline_misses`。1 kHz/290 Hz是目标值；单次丢帧可跳过，持续低于250 Hz须根据实测继续调整。
4. 跟踪日志新增 `ADC均值/峰值`、`采样超期`、`最大唤醒延迟`。若 ADC 读取均值接近或超过 1 ms，先查 GPIO BUSY 事件、SPI 和超时；若读取远低于 1 ms 而采样仍低于 1 kHz，先查线程调度或其他负载。`采样超期`和`最大唤醒延迟`是从启动起累计/最大值；跟踪状态还给出 `psd_sample_age_ms`。
5. v20 曾将 PSD GPIO/SPI 采集放进独立进程；v22 已将运行和设定并行测速都切回单进程线程后端。看门狗在测速切换后重开完整统计窗口，且只在实发速率达标而反馈不足时尝试复位。单次低 RX 不会被当作链路的固有吞吐上限。
6. v21 在连续无效PSD样本确认丢光后清空旧滤波位置；光斑重新出现时先按新位置滤波，仍须完成有效样本确认后才会移动。运动日志中的“近1秒运动命令”直接使用当前滑动窗口计数。
7. v22 的线程后端会与电机通信等任务共享Python主进程，设定频率只是目标；若实测反馈低于250 Hz，跟踪预检仍会阻止启动。
