import os
from datetime import datetime


def _get_log_dir():
    """获取exe所在目录（或脚本所在目录）"""
    import sys
    if getattr(sys, 'frozen', False):
        # 打包为exe时，使用exe文件所在目录
        return os.path.dirname(sys.executable)
    else:
        # 直接运行py脚本时，使用脚本所在目录
        return os.path.dirname(os.path.abspath(__file__))


def _write(filename, content):
    """写入日志文件"""
    path = os.path.join(_get_log_dir(), filename)
    with open(path, 'a', encoding='utf-8') as f:
        f.write(content)


# ─────────────────────────────────────────────
# 日志1：command 通信（发送 & 接收）
# ─────────────────────────────────────────────
def log_command_send(message: str):
    """记录 command 客户端发出的 JSON 包"""
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    _write('log_command.txt',
           f'\n[SEND] {ts}\n{message}\n')


def log_command_recv(message: str):
    """记录 command 客户端收到的 JSON 包"""
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    _write('log_command.txt',
           f'\n[RECV] {ts}\n{message}\n')


# ─────────────────────────────────────────────
# 日志2：实时数据（RealtimePosition 发送）
# ─────────────────────────────────────────────
def log_realtime_send(message: str):
    """记录实时客户端发出的 JSON 包"""
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    _write('log_realtime.txt',
           f'\n[SEND] {ts}\n{message}\n')


# ─────────────────────────────────────────────
# 日志3：wMPS 原始数据
# ─────────────────────────────────────────────
def log_wmps_raw(processor_id: int, raw_data):
    """记录从 wMPS 处理器收到的原始数据。
    TCP模式下 raw_data 是 dict；UDP模式下 raw_data 是原始 JSON 字符串。
    """
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    if isinstance(raw_data, str):
        # UDP模式：直接记录原始JSON包
        _write('log_wmps_raw.txt',
               f'\n[RAW-UDP] {ts}  ProcessorID={processor_id}\n{raw_data}\n')
    else:
        # TCP模式：按通道/发射器格式记录
        lines = [f'\n[RAW] {ts}  ProcessorID={processor_id}']
        for channel, transmitters in raw_data.items():
            for rpm, values in transmitters.items():
                lines.append(
                    f'  channel={channel}  rpm={rpm}'
                    f'  phase1={values[0]:.6f}  phase2={values[1]:.6f}'
                    f'  period={values[2]}  sync_tag={values[3]}'
                )
        _write('log_wmps_raw.txt', '\n'.join(lines) + '\n')


# ─────────────────────────────────────────────
# 初始化：写入会话开始标记
# ─────────────────────────────────────────────
def init_logs():
    """在三个日志文件中写入本次会话的开始标记"""
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    sep = f'\n{"="*60}\n  会话开始: {ts}\n{"="*60}\n'
    for filename in ('log_command.txt', 'log_realtime.txt', 'log_wmps_raw.txt'):
        _write(filename, sep)