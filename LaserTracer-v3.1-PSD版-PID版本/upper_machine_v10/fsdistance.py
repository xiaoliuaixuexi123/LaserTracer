from PySide6.QtCore import (
    QObject, Signal, QThread, Qt, QByteArray, QTimer, Slot
)
import re
import numpy as np
from collections import deque
import time
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
    lpp_ready = Signal(float)          # LPP response
    rolling_result = Signal(dict)     # 滑动窗口实时滤波结果
    error_detected = Signal(str)

    # ------------------------------------------------------------
    def __init__(
        self,
        rude_distance=0,
        buffer_size=1000,
        ref_min=1580.0,
        ref_max=1581.0,
        mad_threshold=3.5
    ):
        super().__init__()

        self.rude_distance = rude_distance
        self.data_buffer = deque(maxlen=buffer_size)

        # 第二通道（稳定参考）识别范围
        self.ref_min = float(ref_min)
        self.ref_max = float(ref_max)
        self.avg_2 = 1580.0

        # ===== 测量级状态 =====
        self.current_ng = None      # 慢变量，只要来一次即可

        # ===== 组级状态 =====
        self.current_fr1 = None     # 每组更新

        self.current_lpp_mm = None
        self.pending_lpp_finalize = False
        self.expected_lpp_commands = 0
        self.lpp_command_count = 0
        self.lpp_values_after_stop = []
        self.raw_ch1 = []           # 第一通道（目标）
        self.raw_ch2 = []           # 第二通道（参考）

        # ===== 统计 =====
        self.group_index = 0
        self.all_group_distances = []

        # ===== 累积平均 + 异常值过滤 =====
        self.all_valid_distances = []   # 累积所有有效值（用于最终均值）
        self.last_valid_distance = None  # 最近一次有效值，供采集按钮使用
        self.mad_threshold = mad_threshold
        self.measurement_active = True
        self.outlier_count = 0

        # 锁定基线：采集前 N 个稳定值后锁定，后续所有值都跟此基线比较
        self.baseline_values = []        # 基线原始值
        self.baseline_median = None      # 锁定后的中位数
        self.baseline_mad = None         # 锁定后的 MAD
        self.baseline_locked = False     # 是否已锁定
        self.baseline_min_count = 30     # 需要多少个值才锁定

    # ------------------------------------------------------------
    @Slot(float)
    def set_rude_distance(self, distance):
        self.rude_distance = distance

    # ------------------------------------------------------------
    def reset_lpp_state(self):
        self.current_lpp_mm = None
        self.pending_lpp_finalize = False
        self.expected_lpp_commands = 0
        self.lpp_command_count = 0
        self.lpp_values_after_stop.clear()

    # ------------------------------------------------------------
    def prepare_lpp_finalize(self, expected_lpp_commands=1):
        self.pending_lpp_finalize = True
        self.expected_lpp_commands = max(1, int(expected_lpp_commands))
        self.lpp_command_count = 0
        self.lpp_values_after_stop.clear()
        self.current_lpp_mm = None

    # ------------------------------------------------------------
    @Slot()
    def finalize_pending_lpp_measurement(self):
        return self._finalize_pending_lpp_measurement(force=True)

    # ------------------------------------------------------------
    def _finalize_pending_lpp_measurement(self, force=False):
        if not self.pending_lpp_finalize:
            return False

        enough_echoes = self.lpp_command_count >= self.expected_lpp_commands
        enough_values = len(self.lpp_values_after_stop) >= self.expected_lpp_commands
        has_old_group_result = bool(self.all_group_distances)
        has_fallback_lpp = self.current_fr1 is not None and self.current_ng is not None

        if self.current_lpp_mm is None and not has_old_group_result and not has_fallback_lpp:
            return False
        if not force and not (enough_echoes or enough_values):
            return False

        self.pending_lpp_finalize = False
        self.finalize_results()
        return True

    # ------------------------------------------------------------
    def _get_lpp_mm(self):
        if self.current_lpp_mm is not None:
            lpp = float(self.current_lpp_mm)
            return lpp if lpp > 0 else None

        if self.current_fr1 is None or self.current_ng is None:
            return None

        c = 3e8
        lpp = c / (2 * self.current_fr1 * 1e6 * self.current_ng) * 1e3
        return lpp if lpp > 0 else None
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

        m_lpp_cmd = re.search(r'Command received:\s*LPP\b', line, re.IGNORECASE)
        if m_lpp_cmd:
            if self.pending_lpp_finalize:
                self.lpp_command_count += 1
            return

        m_lpp = re.search(
            r'\bLPP\b\s*(?:=|:)?\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*(?:mm)?\b',
            line,
            re.IGNORECASE
        )
        if m_lpp:
            try:
                lpp_mm = float(m_lpp.group(1))
            except ValueError:
                return

            self.current_lpp_mm = lpp_mm
            if self.pending_lpp_finalize:
                self.lpp_values_after_stop.append(lpp_mm)
                self._finalize_pending_lpp_measurement(force=False)
            self.lpp_ready.emit(lpp_mm)
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

        lpp = self._get_lpp_mm()
        if lpp is None:
            return

        self.group_index += 1

        # 第一通道平均
        avg_ch1 = float(np.mean(self.raw_ch1))

        # 第二通道平均（仅输出，不参与测距）
        avg_ch2 = float(np.mean(self.raw_ch2)) if self.raw_ch2 else None

        # 粗测展开
        lpp_source = "device" if self.current_lpp_mm is not None else "fr1"
        n = int((self.rude_distance + 4800.0) // lpp)
        distance_offset_mm = 3906.89
        distance = n * lpp + avg_ch1 - distance_offset_mm

        # 保存用于”所有组平均”
        if self.measurement_active:
            self.all_group_distances.append(distance)
            self._update_cumulative_average(distance)

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
            "lpp_source": lpp_source,
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
    def _update_cumulative_average(self, new_value):
        """锁定基线 MAD 异常值检测。
        采集前 N 个稳定值建立基线并锁定，后续所有值都跟锁定基线比较。
        基线不会被后续数据"感染"，跳变及缓慢恢复都会被拒绝。
        """
        # ---- 基线未锁定：积累到足够数量后锁定 ----
        if not self.baseline_locked:
            self.baseline_values.append(new_value)
            self.all_valid_distances.append(new_value)
            self.last_valid_distance = new_value

            if len(self.baseline_values) >= self.baseline_min_count:
                base_arr = np.array(self.baseline_values, dtype=np.float64)
                self.baseline_median = float(np.median(base_arr))
                abs_dev = np.abs(base_arr - self.baseline_median)
                self.baseline_mad = float(np.median(abs_dev))
                if self.baseline_mad < 1e-9:
                    self.baseline_mad = 1e-9
                self.baseline_locked = True
                print(
                    f"[基线锁定] n={len(self.baseline_values)}, "
                    f"median={self.baseline_median:.3f}, "
                    f"MAD={self.baseline_mad:.6f}"
                )

            data_arr = np.array(self.all_valid_distances, dtype=np.float64)
            avg = float(np.mean(data_arr))
            std = float(np.std(data_arr, ddof=1)) if len(data_arr) > 1 else 0.0
            self.rolling_result.emit({
                "cumulative_mean_mm": avg,
                "cumulative_std_mm": std,
                "valid_count": len(data_arr),
                "outlier_total": self.outlier_count,
                "last_value": new_value,
                "filtered": False
            })
            return

        # ---- 基线已锁定：用锁定基线做 MAD 检测 ----
        z = 0.6745 * abs(new_value - self.baseline_median) / self.baseline_mad

        if z > self.mad_threshold:
            self.outlier_count += 1
            data_arr = np.array(self.all_valid_distances, dtype=np.float64)
            avg = float(np.mean(data_arr)) if len(data_arr) else 0.0
            std = float(np.std(data_arr, ddof=1)) if len(data_arr) > 1 else 0.0
            self.rolling_result.emit({
                "cumulative_mean_mm": avg,
                "cumulative_std_mm": std,
                "valid_count": len(data_arr),
                "outlier_total": self.outlier_count,
                "last_value": new_value,
                "filtered": True
            })
        else:
            self.all_valid_distances.append(new_value)
            self.last_valid_distance = new_value
            data_arr = np.array(self.all_valid_distances, dtype=np.float64)
            avg = float(np.mean(data_arr))
            std = float(np.std(data_arr, ddof=1)) if len(data_arr) > 1 else 0.0
            self.rolling_result.emit({
                "cumulative_mean_mm": avg,
                "cumulative_std_mm": std,
                "valid_count": len(data_arr),
                "outlier_total": self.outlier_count,
                "last_value": new_value,
                "filtered": False
            })

    # ------------------------------------------------------------
    def set_measurement_active(self, active: bool):
        self.measurement_active = active

    # ------------------------------------------------------------
    def get_cumulative_mean(self):
        """返回当前累积平均值，供采集按钮使用"""
        if len(self.all_valid_distances) == 0:
            return None
        data = np.array(self.all_valid_distances, dtype=np.float64)
        return {
            "mean": float(np.mean(data)),
            "std": float(np.std(data, ddof=1)) if len(data) > 1 else 0.0,
            "count": len(data),
            "outlier_total": self.outlier_count
        }

    # ------------------------------------------------------------
    def reset_cumulative(self):
        self.all_valid_distances.clear()
        self.last_valid_distance = None
        self.baseline_values.clear()
        self.baseline_median = None
        self.baseline_mad = None
        self.baseline_locked = False
        self.outlier_count = 0

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

        if not self.all_group_distances and not self.all_valid_distances:
            self.reset_lpp_state()
            return

        # The one-shot coordinate solution must use the same MAD-filtered
        # population shown by the rolling result.  Fall back to all groups only
        # when no filtered value exists (for compatibility with short/legacy
        # acquisitions).
        final_distances = (
            self.all_valid_distances
            if self.all_valid_distances
            else self.all_group_distances
        )
        final_mean = float(np.mean(final_distances))
        final_std = (
            float(np.std(final_distances, ddof=1))
            if len(final_distances) > 1 else 0.0
        )

        self.final_result.emit({
            "groups_used": len(final_distances),
            "groups_total": len(self.all_group_distances),
            "outlier_total": self.outlier_count,
            "final_mean_mm": final_mean,
            "final_std_mm": final_std
        })

        # ===== 整次测量结束，统一复位 =====
        self.all_group_distances.clear()
        self.reset_cumulative()
        self.group_index = 0
        self.current_ng = None
        self.current_fr1 = None
        self.raw_ch1.clear()
        self.raw_ch2.clear()
        self.reset_lpp_state()

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
