import numpy as np
from scipy.spatial.transform import Rotation as R

def create_coordinate_system(origin, euler_angles):
    """
    3次元座標系を作成する。
    :param origin: 座標系の原点 (x, y, z)
    :param euler_angles: オイラー角 (rx, ry, rz) [degrees]
    :return: (原点, クォータニオン, 回転行列)
    """
    rotation = R.from_euler('xyz', euler_angles, degrees=True)
    quaternion = rotation.as_quat()
    R_matrix = rotation.as_matrix()
    return np.array(origin), quaternion, R_matrix

def transform_point_and_orientation(point_A, quaternion_A, t, R_matrix):
    """
    座標系A上の点と姿勢を座標系Bへ変換する。
    :param point_A: 座標系A上の点 (x, y, z)
    :param quaternion_A: 座標系Aにおける姿勢 (クォータニオン)
    :param t: 座標系AからBへの並進ベクトル (x, y, z)
    :param R_matrix: 座標系AからBへの回転行列 (3x3)
    :return: (座標系Bにおける点の座標, オイラー角[度], クォータニオン)
    """
    # 点の座標変換: P_B = R * P_A + t
    transformed_point = R_matrix @ point_A + t
    
    # 姿勢の変換
    rotation_A = R.from_quat(quaternion_A)
    rotation_B = R.from_matrix(R_matrix)
    
    # 姿勢の合成を逆順に行う
    rotation_new = rotation_A * rotation_B
    
    # 変換後の姿勢をオイラー角とクォータニオンで表現
    transformed_orientation_euler = rotation_new.as_euler('xyz', degrees=True)
    transformed_orientation_quat = rotation_new.as_quat()
    
    return transformed_point, transformed_orientation_euler, transformed_orientation_quat