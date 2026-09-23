#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手动输入跟踪仪坐标 x y z，输出非正交二维转台 pitch / yaw。

使用方法：
1. 把本文件和下面两个文件放在同一个文件夹：
   - stage123_nonorth_axis_params.json
   - stage123_angle_compensation_table.txt

2. 在“手动输入坐标”区域修改：
   INPUT_X
   INPUT_Y
   INPUT_Z

3. 直接运行本文件。

输出：
    pitch_raw / yaw_raw  : 物理模型反解角
    delta_pitch / delta_yaw : 残差补偿量
    pitch_cmd / yaw_cmd  : 最终给转台的角度
    distance_from_origin : 0点到目标点的几何距离
    optical_s_cmd        : 光束线上到目标点的距离参数
"""

import json
import numpy as np
from pathlib import Path
from functools import lru_cache
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot


# ============================================================
# 手动输入坐标：这里填双发射站 / WMPS 输出的跟踪仪坐标系坐标
# 单位：mm
# ============================================================
INPUT_X = 1631.174459
INPUT_Y = -3712.909634
INPUT_Z = -403.688983


# ============================================================
# 文件路径：默认和本程序在同一个文件夹
# ============================================================
BASE_DIR = Path(__file__).resolve().parent

PARAM_JSON = BASE_DIR / "stage123_nonorth_axis_params1.json"
COMP_TABLE_TXT = BASE_DIR / "stage123_angle_compensation_table1.txt"


# ============================================================
# 基础旋转函数
# ============================================================
def skew(v):
    x, y, z = v
    return np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ], dtype=float)


def rodrigues(axis, theta):
    axis = np.asarray(axis, dtype=float)
    axis = axis / (np.linalg.norm(axis) + 1e-15)

    K = skew(axis)
    I = np.eye(3)

    return I + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def Rz(theta):
    c = np.cos(theta)
    s = np.sin(theta)

    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ], dtype=float)


# ============================================================
# 参数读取
# ============================================================
@lru_cache(maxsize=1)
def load_params_and_compensation():
    if not PARAM_JSON.exists():
        raise FileNotFoundError(f"找不到参数文件: {PARAM_JSON}")

    if not COMP_TABLE_TXT.exists():
        raise FileNotFoundError(f"找不到补偿表文件: {COMP_TABLE_TXT}")

    with open(PARAM_JSON, "r", encoding="utf-8") as f:
        params = json.load(f)

    comp = np.loadtxt(COMP_TABLE_TXT, skiprows=1)

    # comp表列：
    # id raw_pitch raw_yaw meas_pitch meas_yaw delta_pitch delta_yaw forward_err_3d
    raw_pitch_cal = comp[:, 1]
    raw_yaw_cal = comp[:, 2]
    meas_pitch_cal = comp[:, 3]
    meas_yaw_cal = comp[:, 4]
    delta_pitch_cal = comp[:, 5]
    delta_yaw_cal = comp[:, 6]

    return params, raw_pitch_cal, raw_yaw_cal, meas_pitch_cal, meas_yaw_cal, delta_pitch_cal, delta_yaw_cal


def unpack_advanced(x):
    """
    x_opt 参数含义：
    x[0:3]    R_mount 旋转向量，stage坐标 -> 非正交局部坐标
    x[3:5]    pitch_axis0 = normalize([ax, 1, az])
    x[5:8]    pitch轴线上一点，stage坐标，单位mm
    x[8:10]   beam0_dir = normalize([1, by, bz])
    x[10:13]  beam0_point，stage坐标，单位mm
    x[13]     pitch0_deg
    x[14]     yaw0_deg
    x[15]     pitch_scale
    x[16]     yaw_scale
    """
    x = np.asarray(x, dtype=float)

    R_mount = Rot.from_rotvec(x[0:3]).as_matrix()

    ax, az = x[3], x[4]
    pitch_axis0 = np.array([ax, 1.0, az], dtype=float)
    pitch_axis0 = pitch_axis0 / np.linalg.norm(pitch_axis0)

    c_pitch0 = x[5:8].astype(float)

    by, bz = x[8], x[9]
    beam0_dir = np.array([1.0, by, bz], dtype=float)
    beam0_dir = beam0_dir / np.linalg.norm(beam0_dir)

    beam0_point = x[10:13].astype(float)

    pitch0_deg = float(x[13])
    yaw0_deg = float(x[14])
    pitch_scale = float(x[15])
    yaw_scale = float(x[16])

    return (
        R_mount,
        pitch_axis0,
        c_pitch0,
        beam0_dir,
        beam0_point,
        pitch0_deg,
        yaw0_deg,
        pitch_scale,
        yaw_scale
    )


# ============================================================
# 正向光束模型：pitch/yaw -> 光束线 b + s*u
# ============================================================
def ray_local_from_angles(pitch, yaw, params):
    x = np.array(params["x_opt"], dtype=float)

    pitch_sign = params["angle_definition"]["pitch_sign_internal"]
    yaw_sign = params["angle_definition"]["yaw_sign_internal"]

    (
        R_mount,
        pitch_axis0,
        c_pitch0,
        beam0_dir,
        beam0_point,
        pitch0_deg,
        yaw0_deg,
        pitch_scale,
        yaw_scale
    ) = unpack_advanced(x)

    pitch_rad = np.deg2rad(pitch_sign * pitch_scale * (pitch - pitch0_deg))
    yaw_rad = np.deg2rad(yaw_sign * yaw_scale * (yaw - yaw0_deg))

    # 1. 光束先绕真实pitch轴旋转
    Rp = rodrigues(pitch_axis0, pitch_rad)

    b_pitch = c_pitch0 + Rp @ (beam0_point - c_pitch0)
    u_pitch = Rp @ beam0_dir

    # 2. 整个pitch组件再绕yaw轴旋转
    Ryaw = Rz(yaw_rad)

    b_stage = Ryaw @ b_pitch
    u_stage = Ryaw @ u_pitch
    u_stage = u_stage / np.linalg.norm(u_stage)

    # 3. stage坐标转到非正交局部坐标
    b_local = R_mount @ b_stage
    u_local = R_mount @ u_stage
    u_local = u_local / np.linalg.norm(u_local)

    return b_local, u_local


def line_perp_residual_to_point(pitch, yaw, p_local, params):
    """
    目标点到光束线的垂直残差。
    光束线：
        X = b + s*u
    """
    b, u = ray_local_from_angles(pitch, yaw, params)

    w = p_local - b
    s = float(np.dot(w, u))
    perp = w - s * u

    return perp, s, b, u


# ============================================================
# 坐标 -> 原始角度反解
# ============================================================
def ideal_inverse_initial(p_local, params):
    """
    用理想球坐标给优化初值。
    这里只是初值，不是最终结果。
    """
    x = np.array(params["x_opt"], dtype=float)

    pitch_sign = params["angle_definition"]["pitch_sign_internal"]
    yaw_sign = params["angle_definition"]["yaw_sign_internal"]

    (
        R_mount,
        _pitch_axis0,
        _c_pitch0,
        _beam0_dir,
        _beam0_point,
        pitch0_deg,
        yaw0_deg,
        pitch_scale,
        yaw_scale
    ) = unpack_advanced(x)

    q = R_mount.T @ (p_local / (np.linalg.norm(p_local) + 1e-15))

    yaw_right = np.rad2deg(np.arctan2(q[1], q[0]))
    pitch_right = np.rad2deg(np.arctan2(q[2], np.sqrt(q[0]**2 + q[1]**2)))

    pitch_init = pitch0_deg + pitch_right / (pitch_sign * pitch_scale)
    yaw_init = yaw0_deg + yaw_right / (yaw_sign * yaw_scale)

    return pitch_init, yaw_init


def inverse_xyz_to_raw_angles(
    p_local,
    params,
    raw_pitch_cal,
    raw_yaw_cal,
    meas_pitch_cal,
    meas_yaw_cal
):
    """
    输入非正交局部坐标 p_local，反解 pitch_raw/yaw_raw。
    """
    # 用标定角范围外扩作为边界，避免优化跑飞
    p_min = float(np.min(meas_pitch_cal) - 15.0)
    p_max = float(np.max(meas_pitch_cal) + 15.0)
    y_min = float(np.min(meas_yaw_cal) - 15.0)
    y_max = float(np.max(meas_yaw_cal) + 15.0)

    starts = []

    # 初值1：理想球坐标反解
    starts.append(ideal_inverse_initial(p_local, params))

    # 初值2：补偿表里最近的几个raw角度
    # 这里用方向粗略接近的角度点增强收敛稳定性
    target_norm = np.linalg.norm(p_local)
    if target_norm < 1e-12:
        raise ValueError("输入点太接近0点，无法计算指向角。")

    # 用 raw角度补偿表中的点做多初值
    # 因为没有原标定坐标，这里直接取角度空间均值和表中若干代表点
    starts.append((float(np.mean(meas_pitch_cal)), float(np.mean(meas_yaw_cal))))

    idxs = np.linspace(0, len(raw_pitch_cal) - 1, min(8, len(raw_pitch_cal))).astype(int)
    for idx in idxs:
        starts.append((float(meas_pitch_cal[idx]), float(meas_yaw_cal[idx])))

    best = None

    for st_pitch, st_yaw in starts:
        st = np.array([
            np.clip(st_pitch, p_min, p_max),
            np.clip(st_yaw, y_min, y_max)
        ], dtype=float)

        def fun(a):
            pitch, yaw = a
            perp, s, _b, _u = line_perp_residual_to_point(pitch, yaw, p_local, params)

            # 轻微惩罚s<0，避免光束反向瞄准
            neg_s = max(0.0, -s)

            return np.r_[perp, 0.01 * neg_s]

        res = least_squares(
            fun,
            st,
            bounds=([p_min, y_min], [p_max, y_max]),
            method="trf",
            ftol=1e-13,
            xtol=1e-13,
            gtol=1e-13,
            max_nfev=3000,
            x_scale="jac"
        )

        pitch_raw = float(res.x[0])
        yaw_raw = float(res.x[1])

        perp, s, b, u = line_perp_residual_to_point(pitch_raw, yaw_raw, p_local, params)
        aim_error = float(np.linalg.norm(perp))

        cand = {
            "pitch_raw": pitch_raw,
            "yaw_raw": yaw_raw,
            "aim_error_raw_mm": aim_error,
            "s_raw": float(s),
            "ray_point_raw": b + s * u
        }

        if best is None or cand["aim_error_raw_mm"] < best["aim_error_raw_mm"]:
            best = cand

    return best


# ============================================================
# IDW角度残差补偿
# ============================================================
def idw_compensation(
    pitch_raw,
    yaw_raw,
    raw_pitch_cal,
    raw_yaw_cal,
    delta_pitch_cal,
    delta_yaw_cal,
    power=2.0,
    k=8,
    eps=1e-9
):
    pts = np.column_stack([raw_pitch_cal, raw_yaw_cal])
    q = np.array([pitch_raw, yaw_raw], dtype=float)

    dist = np.linalg.norm(pts - q.reshape(1, 2), axis=1)

    if np.min(dist) < 1e-8:
        idx = int(np.argmin(dist))
        return float(delta_pitch_cal[idx]), float(delta_yaw_cal[idx])

    idxs = np.argsort(dist)[:min(k, len(dist))]

    w = 1.0 / (dist[idxs]**power + eps)
    w = w / np.sum(w)

    delta_pitch = float(np.sum(w * delta_pitch_cal[idxs]))
    delta_yaw = float(np.sum(w * delta_yaw_cal[idxs]))

    return delta_pitch, delta_yaw


# ============================================================
# 主流程：跟踪仪坐标 x/y/z -> pitch/yaw
# ============================================================
def xyz_to_pitch_yaw(x, y, z):
    (
        params,
        raw_pitch_cal,
        raw_yaw_cal,
        meas_pitch_cal,
        meas_yaw_cal,
        delta_pitch_cal,
        delta_yaw_cal
    ) = load_params_and_compensation()

    origin = np.array(params["origin_global_tracker"], dtype=float)

    p_tracker = np.array([x, y, z], dtype=float)

    # 跟踪仪坐标 -> 非正交局部坐标
    p_local = p_tracker - origin

    distance_from_origin = float(np.linalg.norm(p_local))

    if distance_from_origin < 1e-9:
        raise ValueError("输入坐标正好是0点，无法输出pitch/yaw。")

    # 第一阶段：物理模型反解原始角度
    raw = inverse_xyz_to_raw_angles(
        p_local,
        params,
        raw_pitch_cal,
        raw_yaw_cal,
        meas_pitch_cal,
        meas_yaw_cal
    )

    pitch_raw = raw["pitch_raw"]
    yaw_raw = raw["yaw_raw"]

    # 第二/第三阶段：IDW残差补偿
    delta_pitch, delta_yaw = idw_compensation(
        pitch_raw,
        yaw_raw,
        raw_pitch_cal,
        raw_yaw_cal,
        delta_pitch_cal,
        delta_yaw_cal,
        power=2.0,
        k=8
    )

    pitch_cmd = pitch_raw + delta_pitch
    yaw_cmd = yaw_raw + delta_yaw

    # 补偿后再计算一次模型指向误差和光束参数
    perp_cmd, s_cmd, b_cmd, u_cmd = line_perp_residual_to_point(
        pitch_cmd,
        yaw_cmd,
        p_local,
        params
    )

    return {
        "input_tracker_xyz": p_tracker,
        "origin_tracker_xyz": origin,
        "local_xyz": p_local,
        "distance_from_origin": distance_from_origin,

        "pitch_raw": pitch_raw,
        "yaw_raw": yaw_raw,

        "delta_pitch": delta_pitch,
        "delta_yaw": delta_yaw,

        "pitch_cmd": pitch_cmd,
        "yaw_cmd": yaw_cmd,

        "aim_error_raw_mm": raw["aim_error_raw_mm"],
        "aim_error_cmd_model_mm": float(np.linalg.norm(perp_cmd)),

        "optical_s_cmd_mm": float(s_cmd)
    }


def main():
    result = xyz_to_pitch_yaw(INPUT_X, INPUT_Y, INPUT_Z)

    print("\n========== 输入坐标 ==========")
    print(f"X = {INPUT_X:.6f} mm")
    print(f"Y = {INPUT_Y:.6f} mm")
    print(f"Z = {INPUT_Z:.6f} mm")

    print("\n========== 坐标转换 ==========")
    print(f"origin_tracker = {result['origin_tracker_xyz']}")
    print(f"local_xyz      = {result['local_xyz']}")
    print(f"distance       = {result['distance_from_origin']:.6f} mm")

    print("\n========== 角度反解结果 ==========")
    print(f"pitch_raw      = {result['pitch_raw']:.9f} deg")
    print(f"yaw_raw        = {result['yaw_raw']:.9f} deg")

    print("\n========== 残差补偿 ==========")
    print(f"delta_pitch    = {result['delta_pitch']:.9f} deg")
    print(f"delta_yaw      = {result['delta_yaw']:.9f} deg")

    print("\n========== 最终给转台的角度 ==========")
    print(f"pitch_cmd      = {result['pitch_cmd']:.9f} deg")
    print(f"yaw_cmd        = {result['yaw_cmd']:.9f} deg")

    print("\n========== 模型检查 ==========")
    print(f"aim_error_raw_mm       = {result['aim_error_raw_mm']:.6f}")
    print(f"aim_error_cmd_model_mm = {result['aim_error_cmd_model_mm']:.6f}")
    print(f"optical_s_cmd_mm       = {result['optical_s_cmd_mm']:.6f}")


if __name__ == "__main__":
    main()
