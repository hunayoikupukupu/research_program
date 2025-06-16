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
    
    # 姿勢の変換: R_B = R * R_A
    rotation_A = R.from_quat(quaternion_A)
    R_A_matrix = rotation_A.as_matrix()
    R_new = R_matrix @ R_A_matrix
    
    # 回転行列からRotationオブジェクトを作成
    rotation_new = R.from_matrix(R_new)
    
    # 変換後の姿勢をオイラー角とクォータニオンで表現
    transformed_orientation_euler = rotation_new.as_euler('xyz', degrees=True)
    transformed_orientation_quat = rotation_new.as_quat()
    
    return transformed_point, transformed_orientation_euler, transformed_orientation_quat

# テストコード
# ① 座標系Aの定義（原点を0,0,0とする）
origin_A = np.array([0, 0, 0])
euler_A = [0, 0, 45]  # 座標系Aは回転なし
_, quaternion_A, R_A = create_coordinate_system(origin_A, euler_A)

# ② 座標系Bの定義（座標系Aから見た相対位置と姿勢）
origin_B = np.array([1, 2, 3])  # 座標系Aから見た座標系Bの原点
euler_B = [0, 0, 0]  # 座標系Aから見た座標系Bの回転（z軸周りに90度）
_, quaternion_B, R_B = create_coordinate_system(origin_B, euler_B)

# ③ 点Pの定義（座標系A上の位置と姿勢）
point_P_A = np.array([5, -5, 5])
euler_P = [10, 20, 120]  # 点Pの姿勢
_, quaternion_P_A, _ = create_coordinate_system(point_P_A, euler_P)

# ④ 座標変換の実行
point_P_B, orientation_P_B_euler, orientation_P_B_quat = transform_point_and_orientation(
    point_P_A, quaternion_P_A, origin_B, R_B
)

# 結果の表示（小数点第4位で四捨五入）
print("\n=== 座標系の定義 ===")
print("座標系A: 原点 =", np.round(origin_A, 4), "回転 =", np.round(euler_A, 4))
print("座標系B: 原点 =", np.round(origin_B, 4), "回転 =", np.round(euler_B, 4))

print("\n=== 変換前 ===")
print("座標系Aにおける点Pの座標:", np.round(point_P_A, 4))
print("座標系Aにおける点Pの姿勢 (roll, pitch, yaw):", np.round(euler_P, 4))
print("座標系Aにおける点Pの姿勢 (クォータニオン):", np.round(quaternion_P_A, 4))

print("\n=== 変換後 ===")
print("座標系Bにおける点Pの座標:", np.round(point_P_B, 4))
print("座標系Bにおける点Pの姿勢 (roll, pitch, yaw):", np.round(orientation_P_B_euler, 4))
print("座標系Bにおける点Pの姿勢 (クォータニオン):", np.round(orientation_P_B_quat, 4))