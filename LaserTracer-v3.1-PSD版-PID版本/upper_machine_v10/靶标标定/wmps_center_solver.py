# -*- coding: utf-8 -*-
"""
WMPS四靶标 -> 角锥(center_point)坐标解算

用途：
1) 用标定文件 1.txt、2.txt、3.txt、4.txt、center_points.txt 标定靶标刚体几何；
2) 实测时输入四个WMPS靶标坐标，输出角锥中心坐标。

数据格式支持：
    id x y z
或：
    x y z

注意：
- 四个WMPS靶标输入顺序必须固定为 1、2、3、4。
- 单位保持一致，一般为 mm。
"""

from __future__ import annotations

import json
from pathlib import Path
from io import StringIO
from typing import Dict, List, Tuple, Any

import numpy as np


# =========================
# 1. 修改这里即可运行
# =========================
BASE_DIR = Path(__file__).resolve().parent

MARKER_FILES = [
    BASE_DIR / "1.txt",   # WMPS靶标1
    BASE_DIR / "2.txt",   # WMPS靶标2
    BASE_DIR / "3.txt",   # WMPS靶标3
    BASE_DIR / "4.txt",   # WMPS靶标4
]
CENTER_FILE = BASE_DIR / "center_points.txt"   # 角锥中心真值/标定值
PARAM_JSON = BASE_DIR / "wmps_center_params.json"

# 实测时把这里改成四个WMPS靶标坐标；顺序必须是1、2、3、4。
# 不想手动输入时保持 None，程序会用标定数据做自检。
NEW_MARKERS_4X3 = None
# 示例：
# NEW_MARKERS_4X3 = np.array([
#     [-604.758383, -3638.064465, -326.499871],  # marker 1
#     [-673.102170, -3597.145253, -321.040884],  # marker 2
#     [-597.305132, -3658.097923, -296.275154],  # marker 3
#     [-614.690212, -3631.548104, -278.544553],  # marker 4
# ], dtype=float)

# 刚体拟合残差超过这个阈值会报警。WMPS若是0.2 mm级，可以设0.5~1.0；
# 若是2 mm级，可以设3~5。
WARN_RIGID_RMS_MM = 1.0


def read_xyz_file(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """
    读取 id x y z 或 x y z 文本文件。
    返回：
        ids: (N,)
        xyz: (N,3)
    """
    text = Path(path).read_text(encoding="utf-8", errors="ignore")
    text = text.replace(",", " ").replace("\t", " ")
    arr = np.genfromtxt(StringIO(text), dtype=float)

    if arr.ndim == 1:
        arr = arr.reshape(1, -1)

    if arr.shape[1] >= 4:
        ids = arr[:, 0].astype(int)
        xyz = arr[:, 1:4].astype(float)
    elif arr.shape[1] == 3:
        ids = np.arange(1, arr.shape[0] + 1, dtype=int)
        xyz = arr[:, 0:3].astype(float)
    else:
        raise ValueError(f"{path} 列数不足，至少需要 x y z 三列。")

    return ids, xyz


def load_calibration_dataset(
    marker_files: List[Path],
    center_file: Path,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    对齐四个靶标与center_point的ID。
    返回：
        common_ids: (N,)
        markers: (N,4,3)
        centers: (N,3)
    """
    marker_data = [read_xyz_file(p) for p in marker_files]
    center_ids, centers_all = read_xyz_file(center_file)

    id_sets = [set(ids.tolist()) for ids, _ in marker_data]
    id_sets.append(set(center_ids.tolist()))
    common_ids = sorted(set.intersection(*id_sets))

    if len(common_ids) < 4:
        raise ValueError("可用于标定的公共ID少于4个，请检查1/2/3/4和center_points文件。")

    def pick_by_ids(ids: np.ndarray, xyz: np.ndarray, wanted: List[int]) -> np.ndarray:
        table = {int(i): xyz[k] for k, i in enumerate(ids)}
        return np.array([table[int(i)] for i in wanted], dtype=float)

    markers = np.stack(
        [pick_by_ids(ids, xyz, common_ids) for ids, xyz in marker_data],
        axis=1,
    )
    centers = pick_by_ids(center_ids, centers_all, common_ids)
    return np.array(common_ids, dtype=int), markers, centers


def kabsch(template: np.ndarray, measured: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    最小二乘刚体配准。
    求 R,t，使：
        measured ≈ template @ R.T + t
    输入：
        template: (M,3)
        measured: (M,3)
    输出：
        R: (3,3)
        t: (3,)
    """
    template = np.asarray(template, dtype=float)
    measured = np.asarray(measured, dtype=float)

    ct = template.mean(axis=0)
    cm = measured.mean(axis=0)
    A = template - ct
    B = measured - cm

    H = A.T @ B
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 防止镜像反射
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T

    t = cm - ct @ R.T
    return R, t


def calc_affine_weights(markers: np.ndarray, centers: np.ndarray) -> np.ndarray:
    """
    辅助模型：center = w1*p1 + w2*p2 + w3*p3 + w4*p4, 且 sum(w)=1。
    这个模型对刚体平移/旋转天然不变，可作为Kabsch方法的交叉检查。
    """
    n = markers.shape[0]
    A = np.vstack([markers[:, :, k] for k in range(3)])  # (3N,4)
    b = np.hstack([centers[:, k] for k in range(3)])     # (3N,)

    ATA = A.T @ A
    ATb = A.T @ b
    K = np.block([
        [ATA, np.ones((4, 1))],
        [np.ones((1, 4)), np.zeros((1, 1))],
    ])
    rhs = np.r_[ATb, 1.0]
    sol = np.linalg.solve(K, rhs)
    return sol[:4]


def pair_distance_stats(markers: np.ndarray) -> Dict[str, Dict[str, float]]:
    """统计四个靶标之间的固定边长。"""
    out: Dict[str, Dict[str, float]] = {}
    for a in range(4):
        for b in range(a + 1, 4):
            d = np.linalg.norm(markers[:, a, :] - markers[:, b, :], axis=1)
            key = f"{a+1}-{b+1}"
            out[key] = {
                "mean": float(np.mean(d)),
                "std": float(np.std(d, ddof=0)),
                "min": float(np.min(d)),
                "max": float(np.max(d)),
            }
    return out


def calibrate(markers: np.ndarray, centers: np.ndarray) -> Dict[str, Any]:
    """
    标定刚体模板和center_point在模板坐标系下的位置。
    """
    if markers.shape[1:] != (4, 3):
        raise ValueError("markers必须是(N,4,3)。")

    # 用第一个姿态初始化模板，然后做一次广义Procrustes平均
    template = markers[0].copy()

    for _ in range(20):
        local_markers = []
        for i in range(markers.shape[0]):
            R, t = kabsch(template, markers[i])
            local_markers.append((markers[i] - t) @ R)  # 转回模板坐标系
        new_template = np.mean(local_markers, axis=0)
        if np.linalg.norm(new_template - template) < 1e-12:
            template = new_template
            break
        template = new_template

    # 将每个姿态的center转回模板坐标系，取平均作为固定center位置
    local_centers = []
    for i in range(markers.shape[0]):
        R, t = kabsch(template, markers[i])
        local_centers.append((centers[i] - t) @ R)
    local_centers = np.array(local_centers)
    center_local = np.mean(local_centers, axis=0)

    # 标定误差
    pred_centers = []
    rigid_marker_rms = []
    for i in range(markers.shape[0]):
        R, t = kabsch(template, markers[i])
        pred_centers.append(center_local @ R.T + t)
        pred_markers = template @ R.T + t
        rigid_marker_rms.append(np.sqrt(np.mean(np.sum((pred_markers - markers[i]) ** 2, axis=1))))

    pred_centers = np.array(pred_centers)
    center_err = pred_centers - centers
    center_err_norm = np.linalg.norm(center_err, axis=1)

    affine_weights = calc_affine_weights(markers, centers)
    affine_pred = np.einsum("j,njk->nk", affine_weights, markers)
    affine_err_norm = np.linalg.norm(affine_pred - centers, axis=1)

    params: Dict[str, Any] = {
        "method": "rigid_template_kabsch",
        "unit": "mm",
        "marker_order": [1, 2, 3, 4],
        "template_points_4x3": template.tolist(),
        "center_local_3": center_local.tolist(),
        "pair_distance_stats": pair_distance_stats(markers),
        "calibration_error": {
            "num_poses": int(markers.shape[0]),
            "center_mean_mm": float(np.mean(center_err_norm)),
            "center_rms_mm": float(np.sqrt(np.mean(center_err_norm ** 2))),
            "center_max_mm": float(np.max(center_err_norm)),
            "rigid_marker_rms_mean_mm": float(np.mean(rigid_marker_rms)),
            "rigid_marker_rms_max_mm": float(np.max(rigid_marker_rms)),
        },
        "affine_check": {
            "weights": affine_weights.tolist(),
            "center_mean_mm": float(np.mean(affine_err_norm)),
            "center_rms_mm": float(np.sqrt(np.mean(affine_err_norm ** 2))),
            "center_max_mm": float(np.max(affine_err_norm)),
        },
    }
    return params


def solve_center_from_four_markers(
    new_markers_4x3: np.ndarray,
    params: Dict[str, Any],
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """
    输入当前四个WMPS靶标坐标，输出角锥center_point坐标。
    """
    measured = np.asarray(new_markers_4x3, dtype=float)
    if measured.shape != (4, 3):
        raise ValueError("new_markers_4x3必须是(4,3)，顺序为靶标1、2、3、4。")

    template = np.array(params["template_points_4x3"], dtype=float)
    center_local = np.array(params["center_local_3"], dtype=float)

    R, t = kabsch(template, measured)
    pred_markers = template @ R.T + t
    residuals = pred_markers - measured
    residual_norm = np.linalg.norm(residuals, axis=1)
    rigid_rms = float(np.sqrt(np.mean(residual_norm ** 2)))

    center = center_local @ R.T + t

    # 辅助：仿射权重交叉检查
    affine_center = None
    affine_delta = None
    if "affine_check" in params and "weights" in params["affine_check"]:
        w = np.array(params["affine_check"]["weights"], dtype=float)
        affine_center = np.einsum("j,jk->k", w, measured)
        affine_delta = float(np.linalg.norm(center - affine_center))

    diagnostics: Dict[str, Any] = {
        "rigid_marker_residual_norm_mm": residual_norm.tolist(),
        "rigid_marker_rms_mm": rigid_rms,
        "rotation_matrix": R.tolist(),
        "translation": t.tolist(),
        "affine_center_check": None if affine_center is None else affine_center.tolist(),
        "rigid_vs_affine_delta_mm": affine_delta,
    }
    return center, diagnostics


def save_params(params: Dict[str, Any], path: Path) -> None:
    path.write_text(json.dumps(params, ensure_ascii=False, indent=2), encoding="utf-8")


def load_params(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def print_params_summary(params: Dict[str, Any]) -> None:
    e = params["calibration_error"]
    a = params["affine_check"]

    print("\n========== 标定结果 ==========")
    print(f"标定姿态数: {e['num_poses']}")
    print(f"Kabsch刚体模型 center误差: mean={e['center_mean_mm']:.6f} mm, "
          f"RMS={e['center_rms_mm']:.6f} mm, max={e['center_max_mm']:.6f} mm")
    print(f"四靶标刚体拟合残差: mean RMS={e['rigid_marker_rms_mean_mm']:.6f} mm, "
          f"max RMS={e['rigid_marker_rms_max_mm']:.6f} mm")
    print(f"仿射权重校验 center误差: mean={a['center_mean_mm']:.6f} mm, "
          f"RMS={a['center_rms_mm']:.6f} mm, max={a['center_max_mm']:.6f} mm")
    print("仿射权重 w1,w2,w3,w4 =", np.array(a["weights"]))

    print("\n四个靶标之间的标定边长/mm：")
    for key, s in params["pair_distance_stats"].items():
        print(f"  {key}: mean={s['mean']:.6f}, std={s['std']:.6f}, "
              f"min={s['min']:.6f}, max={s['max']:.6f}")


def main() -> None:
    ids, markers, centers = load_calibration_dataset(MARKER_FILES, CENTER_FILE)
    params = calibrate(markers, centers)
    save_params(params, PARAM_JSON)
    print_params_summary(params)
    print(f"\n参数已保存: {PARAM_JSON}")

    if NEW_MARKERS_4X3 is None:
        # 自检：用第一组标定数据反算center
        print("\n========== 自检：用第1组四靶标反算center ==========")
        test_markers = markers[0]
        true_center = centers[0]
    else:
        print("\n========== 实测解算 ==========")
        test_markers = np.asarray(NEW_MARKERS_4X3, dtype=float)
        true_center = None

    center, diag = solve_center_from_four_markers(test_markers, params)

    print("解算角锥center_point/mm:")
    print(f"  X = {center[0]:.6f}")
    print(f"  Y = {center[1]:.6f}")
    print(f"  Z = {center[2]:.6f}")

    if true_center is not None:
        err = center - true_center
        print("对应真值center_point/mm:")
        print(f"  X = {true_center[0]:.6f}")
        print(f"  Y = {true_center[1]:.6f}")
        print(f"  Z = {true_center[2]:.6f}")
        print(f"误差/mm: dX={err[0]:.6f}, dY={err[1]:.6f}, dZ={err[2]:.6f}, "
              f"|d|={np.linalg.norm(err):.6f}")

    print(f"四靶标刚体拟合RMS = {diag['rigid_marker_rms_mm']:.6f} mm")
    print("单个靶标刚体残差/mm =", np.array(diag["rigid_marker_residual_norm_mm"]))

    if diag["rigid_marker_rms_mm"] > WARN_RIGID_RMS_MM:
        print("\n[警告] 四靶标刚体拟合残差偏大，可能存在：")
        print("  1) 1/2/3/4靶标顺序接错；")
        print("  2) 某个WMPS坐标跳点；")
        print("  3) 靶标板发生形变或安装松动；")
        print("  4) 当前WMPS测量噪声明显大于标定噪声。")

    if diag["rigid_vs_affine_delta_mm"] is not None:
        print(f"Kabsch解 与 仿射权重解 差值 = {diag['rigid_vs_affine_delta_mm']:.6f} mm")


if __name__ == "__main__":
    main()
