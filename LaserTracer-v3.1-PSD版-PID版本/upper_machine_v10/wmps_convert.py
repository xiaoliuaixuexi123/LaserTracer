from __future__ import annotations

import argparse
import math
from itertools import permutations
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

# =====================================================================
# 硬编码标定参数（由 靶标标定.txt 标定得到，无需再读标定文件）
# 模型: C = A + u*(B-A) + v*(D-A) + w*n,  n = normalize((B-A) x (D-A))
# =====================================================================
_CALIB_U = 0.327324997717
_CALIB_V = 0.342148365515
_CALIB_W = 0.065513105060
_CALIB_PARAM = (_CALIB_U, _CALIB_V, _CALIB_W)

# 参考组局部几何特征 [|AB|, |AD|, |BD|, cos∠A]，用于自动统一顶点顺序
_REF_FEATURES = [103.32321455278262, 103.44679673181923, 103.77456500233514, 0.49622539033848073]

# =====================================================================
# 内部向量/几何工具
# =====================================================================

def _vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _vec_scale(v, s):
    return [v[0] * s, v[1] * s, v[2] * s]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _norm(v):
    return math.sqrt(_dot(v, v))


def _normalize(v):
    n = _norm(v)
    if n < 1e-12:
        raise ValueError("三个顶点几乎共线，无法构造法向量。")
    return [v[0] / n, v[1] / n, v[2] / n]


def _compute_local_features(A, B, D):
    AB = _vec_sub(B, A)
    AD = _vec_sub(D, A)
    BD = _vec_sub(D, B)
    lab = _norm(AB)
    lad = _norm(AD)
    lbd = _norm(BD)
    cos_A = _dot(AB, AD) / (lab * lad)
    return [lab, lad, lbd, cos_A]


def _feature_distance(f1, f2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(f1, f2)))


def _all_vertex_orders(vertices):
    return [list(p) for p in permutations(vertices, 3)]


def _align_vertices_to_reference(vertices, ref_features):
    best_order = None
    best_dist = float("inf")
    for order in _all_vertex_orders(vertices):
        A, B, D = order
        feat = _compute_local_features(A, B, D)
        d = _feature_distance(feat, ref_features)
        if d < best_dist:
            best_dist = d
            best_order = order
    return best_order


def _compute_center_from_vertices(A, B, D, param):
    u, v, w = param
    BA = _vec_sub(B, A)
    DA = _vec_sub(D, A)
    n_vec = _normalize(_cross(BA, DA))
    C = _vec_add(
        A,
        _vec_add(
            _vec_add(_vec_scale(BA, u), _vec_scale(DA, v)),
            _vec_scale(n_vec, w),
        ),
    )
    return C


# =====================================================================
# 公开 API
# =====================================================================

def convert_vertices_to_center(v0, v1, v2):
    """将三个顶点坐标转换为中心坐标（使用硬编码标定参数）。

    参数可以是 (x,y,z) 元组或 [x,y,z] 列表，顺序不限，
    函数内部会自动对齐到参考组的顶点顺序。
    """
    vertices = [list(v0), list(v1), list(v2)]
    ordered = _align_vertices_to_reference(vertices, _REF_FEATURES)
    A, B, D = ordered
    C = _compute_center_from_vertices(A, B, D, _CALIB_PARAM)
    return (float(C[0]), float(C[1]), float(C[2]))


# =====================================================================
# 批量处理（离线使用）
# =====================================================================

VALID_VERTEX_IDS = ("0", "2", "4")


def _parse_coord_line(line: str) -> Tuple[str, float, float, float] | None:
    line = line.strip()
    if not line:
        return None
    parts = [p.strip() for p in line.split(",") if p.strip() != ""]
    if len(parts) < 4:
        return None
    vid = parts[0]
    if vid not in VALID_VERTEX_IDS:
        return None
    try:
        x = float(parts[1])
        y = float(parts[2])
        z = float(parts[3])
    except ValueError:
        return None
    return vid, x, y, z


def average_vertices_in_folder(point_dir: Path) -> Tuple[Dict[str, Tuple[float, float, float]], Dict[str, int], int]:
    sums = {"0": [0.0, 0.0, 0.0], "2": [0.0, 0.0, 0.0], "4": [0.0, 0.0, 0.0]}
    counts = {"0": 0, "2": 0, "4": 0}

    coord_files = sorted(point_dir.rglob("*Coord.txt"))
    for file_path in coord_files:
        with file_path.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                parsed = _parse_coord_line(line)
                if parsed is None:
                    continue
                vid, x, y, z = parsed
                sums[vid][0] += x
                sums[vid][1] += y
                sums[vid][2] += z
                counts[vid] += 1

    missing = [vid for vid in VALID_VERTEX_IDS if counts[vid] == 0]
    if missing:
        raise ValueError(f"{point_dir} 缺少顶点数据: {','.join(missing)}")

    avg_vertices: Dict[str, Tuple[float, float, float]] = {}
    for vid in VALID_VERTEX_IDS:
        c = counts[vid]
        avg_vertices[vid] = (sums[vid][0] / c, sums[vid][1] / c, sums[vid][2] / c)

    return avg_vertices, counts, len(coord_files)


def convert_single_point(point_dir: Path) -> Dict[str, object]:
    avg_vertices, counts, file_count = average_vertices_in_folder(point_dir)
    cx, cy, cz = convert_vertices_to_center(
        avg_vertices["0"], avg_vertices["2"], avg_vertices["4"]
    )
    return {
        "point": point_dir.name,
        "file_count": file_count,
        "count_0": counts["0"],
        "count_2": counts["2"],
        "count_4": counts["4"],
        "center": (cx, cy, cz),
        "avg_vertices": avg_vertices,
    }


def convert_points(wmps_root: Path, points: Iterable[int]) -> List[Dict[str, object]]:
    results: List[Dict[str, object]] = []
    for pid in points:
        point_dir = wmps_root / str(pid)
        if not point_dir.exists():
            continue
        result = convert_single_point(point_dir)
        result["point"] = pid
        results.append(result)
    return results


def _write_all_points_output(output_file: Path, results: List[Dict[str, object]]) -> None:
    output_file.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "wmps 1-15 converted coordinates",
        f"u={_CALIB_U:.12f}, v={_CALIB_V:.12f}, w={_CALIB_W:.12f}",
        "",
        "point\tfiles\tcount_0\tcount_2\tcount_4\tconverted_x\tconverted_y\tconverted_z",
    ]
    for r in sorted(results, key=lambda x: int(x["point"])):
        cx, cy, cz = r["center"]
        lines.append(
            f"{r['point']}\t{r['file_count']}\t{r['count_0']}\t{r['count_2']}\t{r['count_4']}"
            f"\t{cx:.6f}\t{cy:.6f}\t{cz:.6f}"
        )
    output_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_single_point_output(output_file: Path, result: Dict[str, object]) -> None:
    output_file.parent.mkdir(parents=True, exist_ok=True)
    cx, cy, cz = result["center"]
    lines = [
        "wmps single point converted coordinate",
        f"point={result['point']}",
        f"u={_CALIB_U:.12f}, v={_CALIB_V:.12f}, w={_CALIB_W:.12f}",
        f"files={result['file_count']}, count_0={result['count_0']}, count_2={result['count_2']}, count_4={result['count_4']}",
        f"converted_x={cx:.6f}",
        f"converted_y={cy:.6f}",
        f"converted_z={cz:.6f}",
    ]
    output_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    base_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description="使用硬编码标定参数将 wmps 顶点数据换算为坐标")
    parser.add_argument("--wmps-root", type=Path, default=base_dir / "wmps")
    parser.add_argument(
        "--all-output",
        type=Path,
        default=base_dir / "txt" / "wmps_1-15_换算坐标.txt",
        help="1-15 点批量换算输出",
    )
    parser.add_argument("--point-dir", type=Path, default=None, help="单点目录")
    parser.add_argument(
        "--point-output",
        type=Path,
        default=base_dir / "txt" / "wmps_single_换算坐标.txt",
        help="单点换算输出",
    )

    args = parser.parse_args()

    print(f"标定系数(硬编码): u={_CALIB_U:.12f}, v={_CALIB_V:.12f}, w={_CALIB_W:.12f}")

    results = convert_points(args.wmps_root, range(1, 16))
    _write_all_points_output(args.all_output, results)
    print(f"已生成批量换算: {args.all_output}")

    if args.point_dir is not None:
        one = convert_single_point(args.point_dir)
        _write_single_point_output(args.point_output, one)
        cx, cy, cz = one["center"]
        print(f"单点换算坐标 ({args.point_dir}): {cx:.6f}, {cy:.6f}, {cz:.6f}")
        print(f"已生成单点换算: {args.point_output}")


if __name__ == "__main__":
    main()
