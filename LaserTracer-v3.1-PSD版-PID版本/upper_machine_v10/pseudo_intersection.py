from numpy import arctan2, sqrt, cos, sin, pi, array
import numpy as np
def rotation_to_euler(rotation):
    """
    generate rotation angle x, y, z  by rotation matrix.
    R = Rz * Ry * Rx
    :param rotation: A (3, 3) matrix for rotation.
    :return angle_x: A float for angle x.
    :return angle_y: A float for angle y.
    :return angle_z: A float for angle z.
    """
    angle_x = arctan2(rotation[2, 1], rotation[2, 2])
    angle_y = arctan2(-rotation[2, 0], sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0]))
    angle_z = arctan2(rotation[1, 0], rotation[0, 0])
    return angle_x, angle_y, angle_z


def euler_to_rotation(angle_x, angle_y, angle_z):
    """
    generate rotation matrix by x, y ,z.
    R = Rz * Ry * Rx
    :param angle_x: A float for angle x.
    :param angle_y: A float for angle y.
    :param angle_z: A float for angle z.
    :return rotation: A (3, 3) matrix for rotation.
    """
    rotation_x = array([[1, 0, 0],
                        [0, cos(angle_x), -sin(angle_x)],
                        [0, sin(angle_x), cos(angle_x)]])
    rotation_y = array([[cos(angle_y), 0, sin(angle_y)],
                        [0, 1, 0],
                        [-sin(angle_y), 0, cos(angle_y)]])
    rotation_z = array([[cos(angle_z), -sin(angle_z), 0],
                        [sin(angle_z), cos(angle_z), 0],
                        [0, 0, 1]])
    rotation = rotation_z.dot(rotation_y.dot(rotation_x))
    return rotation


def calculate_rigid_transform_svd(local_points, world_points):
    """
    利用 SVD (Kabsch算法) 计算从局部坐标系到世界坐标系的刚体变换矩阵。
    等式: P_world = R * P_local + t

    :param local_points: Nx3 的 numpy 数组，表示靶标 XML 中的局部坐标
    :param world_points: Nx3 的 numpy 数组，表示 wMPS 测量到的世界坐标
    :return: (R, t) -> 3x3 的旋转矩阵 R 和形状为 (3,) 的平移向量 t
    """
    assert len(local_points) == len(world_points)
    if len(local_points) < 3:
        raise ValueError("至少需要3个点才能进行完整的 6DOF SVD 解算")

    A = np.array(local_points)
    B = np.array(world_points)

    # 1. 计算质心 (Centroids)
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # 2. 去质心化
    AA = A - centroid_A
    BB = B - centroid_B

    # 3. 计算协方差矩阵 H
    H = AA.T @ BB

    # 4. SVD 分解
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 5. 反射情况处理 (防止镜像翻转)
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    # 6. 计算平移向量 t
    t = centroid_B - R @ centroid_A

    return R, t