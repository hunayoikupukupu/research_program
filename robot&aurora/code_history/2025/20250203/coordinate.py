import numpy as np
from scipy.spatial.transform import Rotation as R

def create_coordinate_system(origin, euler_angles):
    """
    3次元座標系を作成する。
    :param origin: 座標系の原点 (x, y, z)
    :param euler_angles: オイラー角 (rx, ry, rz) [degrees]
    :return: (原点, クォータニオン)
    """
    quaternion = R.from_euler('xyz', euler_angles, degrees=True).as_quat()
    return np.array(origin), quaternion

def transform_point_and_orientation(point_A, quaternion_A, origin_B, quaternion_B):
    """
    座標系A上の点と姿勢を座標系Bへ変換する。
    :param point_A: 座標系A上の点 (x, y, z)
    :param quaternion_A: 座標系Aにおける姿勢 (クォータニオン)
    :param origin_B: 座標系Bの原点
    :param quaternion_B: 座標系Bのクォータニオン
    :return: (座標系Bにおける点Pの座標, 座標系Bにおける姿勢 (クォータニオン))
    """
    rotation_B = R.from_quat(quaternion_B)
    transformed_point = rotation_B.apply(point_A - origin_B)
    
    rotation_A = R.from_quat(quaternion_A)
    transformed_orientation = R.from_quat(quaternion_B) * rotation_A
    transformed_orientation_euler = transformed_orientation.as_euler('xyz', degrees=True)
    transformed_orientation_quat = transformed_orientation.as_quat()
    
    return transformed_point, transformed_orientation_euler, transformed_orientation_quat

# ① 座標系Aの作成（デフォルト原点）
origin_A = np.array([10, -10, -5])
euler_A = [0, 0, 0]  # 座標系Aは回転なし
_, quaternion_A = create_coordinate_system(origin_A, euler_A)

# ② 座標系Bの作成（並進ベクトルとオイラー角を指定）
translation_B = np.array([0, 0, 0])  # Bの原点
rotation_B_angles = [90, 90, 90]  # x, y, z軸回転角度（度）
origin_B, quaternion_B = create_coordinate_system(translation_B, rotation_B_angles)

# ③ 座標系A上の点Pと姿勢
point_P_A = np.array([5, -5, 5])
quaternion_P_A = R.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()

# 点Pの座標と姿勢を座標系Bへ変換
point_P_B, orientation_P_B_euler, orientation_P_B_quat = transform_point_and_orientation(point_P_A, quaternion_P_A, origin_B, quaternion_B)

print("座標系Aにおける点Pの座標:", point_P_A)
print("座標系Aにおける点Pの姿勢 (roll, pitch, yaw):", R.from_quat(quaternion_P_A).as_euler('xyz', degrees=True))
print("座標系Aにおける点Pの姿勢 (クォータニオン):", quaternion_P_A)
print("座標系Bにおける点Pの座標:", point_P_B)
print("座標系Bにおける点Pの姿勢 (roll, pitch, yaw):", orientation_P_B_euler)
print("座標系Bにおける点Pの姿勢 (クォータニオン):", orientation_P_B_quat)
