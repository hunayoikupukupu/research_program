# 同次変換、逆行列、SVDなどのユーティリティ関数

import numpy as np
from scipy.spatial.transform import Rotation as R

# 同次変換行列を扱うクラス
class Transform:
    def __init__(self, R_mat=None, t=None):
        """R_mat: (3,3) 回転行列, t: (3,) 並進ベクトル"""
        self.R = R_mat if R_mat is not None else np.eye(3)
        self.t = t if t is not None else np.zeros(3)

    @classmethod
    def from_quat(cls, quat, t=None):
        """クォータニオン + 並進から生成"""
        R_mat = R.from_quat(quat).as_matrix()
        return cls(R_mat, t)

    @classmethod
    def from_euler(cls, euler, t=None, seq="xyz", degrees=True):
        """オイラー角 + 並進から生成"""
        R_mat = R.from_euler(seq, euler, degrees=degrees).as_matrix()
        return cls(R_mat, t)

    @classmethod
    def from_matrix(cls, T):
        """4×4同次変換行列から生成"""
        R_mat = T[:3, :3]
        t = T[:3, 3]
        return cls(R_mat, t)

    @property
    def matrix(self):
        """4×4の同次変換行列を返す"""
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.t
        return T

    def inv(self):
        """逆変換を返す"""
        R_inv = self.R.T
        t_inv = -R_inv @ self.t
        return Transform(R_inv, t_inv)

    def __matmul__(self, other):
        """@演算子で座標変換を合成可能"""
        if not isinstance(other, Transform):
            raise TypeError("Transform同士でのみ@演算子を使用できます。")
        R_new = self.R @ other.R
        t_new = self.R @ other.t + self.t
        return Transform(R_new, t_new)

    def __repr__(self):
        return f"Transform(t={self.t}, R=\n{self.R})"


# csvファイルから点群データを作成する関数
def load_csv_data(file_path):
    """
    CSVファイルから点群データを読み込む
    file_path: CSVファイルのパス
    戻り値: Transformオブジェクトのリスト (T_arm_from_robot, T_sensor_from_aurora)
    """

    try:
        csv_data = np.loadtxt(file_path, skiprows=1, delimiter=',')
    except Exception as e:
        print(f"Error loading CSV data: {e}")
        return None
    
    T_arm_from_robot_list = []
    T_sensor_from_aurora_list = []

    for data in csv_data:
        robot_x, robot_y, robot_z = data[0], data[1], data[2]
        robot_rx, robot_ry, robot_rz = data[3], data[4], data[5]
        aurora_x, aurora_y, aurora_z = data[6], data[7], data[8]
        aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w = data[9], data[10], data[11], data[12]

        # arm_from_robotの同次変換行列を作成
        t_arm_from_robot = np.array([robot_x, robot_y, robot_z])
        R_matrix_robot = R.from_rotvec([robot_rx, robot_ry, robot_rz], degrees=True).as_matrix()
        T_arm_from_robot = Transform(R_matrix_robot, t_arm_from_robot).matrix
        T_arm_from_robot_list.append(T_arm_from_robot)

        # sensor_from_auroraの同次変換行列を作成
        t_sensor_from_aurora = np.array([aurora_x, aurora_y, aurora_z])
        R_matrix_aurora = R.from_quat([aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w]).as_matrix()
        T_sensor_from_aurora = Transform(R_matrix_aurora, t_sensor_from_aurora).matrix
        T_sensor_from_aurora_list.append(T_sensor_from_aurora)

    return T_arm_from_robot_list, T_sensor_from_aurora_list

# T_arm_from_robotを計算する関数
def compute_T_arm_from_robot(t_sensor_from_aurora, quaternion_sensor_from_aurora,
                             T_aurora_from_robot,
                             T_arm_from_sensor):
    """
    T_arm_from_robotを計算する関数
    t_sensor_from_aurora: センサーのオイラー角 [x, y, z]
    quaternion_sensor_from_aurora: センサーのクォータニオン [x, y, z, w]
    T_aurora_from_robot: 4x4同次変換行列
    T_arm_from_sensor: 4x4同次変換行列
    戻り値: T_arm_from_robot: 4x4同次変換行列
    """
    # T_sensor_from_auroraを作成
    R_sensor_from_aurora = R.from_quat(quaternion_sensor_from_aurora).as_matrix()
    T_sensor_from_aurora_transform = Transform(R_sensor_from_aurora, t_sensor_from_aurora)

    # T_sensor_from_robotを求める
    T_aurora_from_robot_transform = Transform.from_matrix(T_aurora_from_robot)
    T_sensor_from_robot_transform = T_aurora_from_robot_transform @ T_sensor_from_aurora_transform

    # T_arm_from_robotを求める
    T_arm_from_sensor_transform = Transform.from_matrix(T_arm_from_sensor)
    T_arm_from_robot_transform = T_sensor_from_robot_transform @ T_arm_from_sensor_transform

    return T_arm_from_robot_transform.matrix


# 2つの同次変換行列の差分を計算する関数
def compute_transform_difference(T1, T2):
    """
    2つの同次変換行列の差分を計算する関数
    T1, T2: 4x4同次変換行列
    戻り値:
        delta_t: 並進差分ベクトル (x, y, z)
        delta_rotvec: 回転差分ベクトル (軸×角度)
        delta_t_norm: 並進差の大きさ
        delta_angle: 回転角の大きさ（ラジアン）
    """

    # Transformオブジェクトに変換
    T_transform1 = Transform.from_matrix(T1)
    T_transform2 = Transform.from_matrix(T2)

    # 並進成分
    t_transform1 = T_transform1.t
    t_transform2 = T_transform2.t
    delta_t = t_transform2 - t_transform1
    delta_t_norm = np.linalg.norm(delta_t)

    # 回転成分
    R1 = T_transform1.R
    R2 = T_transform2.R
    R_diff = R1.T @ R2
    delta_rotvec = R.from_matrix(R_diff).as_rotvec()
    delta_angle_rad = np.linalg.norm(delta_rotvec)
    delta_angle_deg = np.degrees(delta_angle_rad)

    print(f"並進差分ベクトル: {delta_t}, 大きさ: {delta_t_norm}")
    print(f"回転差分ベクトル: {delta_rotvec}, 回転角の大きさ: {delta_angle_deg:.3f} 度")


    return delta_t, delta_rotvec, delta_t_norm, delta_angle_deg
