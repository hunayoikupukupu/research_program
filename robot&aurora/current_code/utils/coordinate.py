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


# def transform_point_and_orientation(point_A, quaternion_A, origin_B, quaternion_B):
#     """
#     座標系A上の点と姿勢を座標系Bへ変換する。
#     :param point_A: 座標系A上の点 (x, y, z)
#     :param quaternion_A: 座標系Aにおける姿勢 (クォータニオン)
#     :param origin_B: 座標系Bの原点
#     :param quaternion_B: 座標系Bのクォータニオン
#     :return: (座標系Bにおける点Pの座標, 座標系Bにおける姿勢 (クォータニオン))
#     """
#     rotation_B = R.from_quat(quaternion_B)
#     transformed_point = rotation_B.apply(point_A + origin_B)
    
#     rotation_A = R.from_quat(quaternion_A)
#     transformed_orientation = rotation_A * rotation_B.inv()
#     transformed_orientation_euler = transformed_orientation.as_euler('xyz', degrees=True)
#     transformed_orientation_quat = transformed_orientation.as_quat()
    
#     return transformed_point, transformed_orientation_euler, transformed_orientation_quat

def transform_point_and_orientation(point_A, quaternion_A, origin_B, quaternion_B):
    """
    座標系A上の点と姿勢を座標系Bへ変換する。
    :param point_A: 座標系A上の点 (x, y, z)
    :param quaternion_A: 座標系Aにおける姿勢 (クォータニオン)
    :param origin_B: 座標系Bの原点
    :param quaternion_B: 座標系Bのクォータニオン
    :return: (座標系Bにおける点Pの座標, 座標系Bにおける姿勢 (クォータニオン))
    """
    # クォータニオンの逆変換を適用
    rotation_B = R.from_quat(quaternion_B)
    transformed_point = rotation_B.apply(point_A + origin_B)

    # y, z 軸を反転（座標系の定義に応じて）
    transformed_point[1] *= -1  
    transformed_point[2] *= -1  

    # 姿勢変換
    rotation_A = R.from_quat(quaternion_A)
    transformed_orientation = rotation_A * rotation_B
    transformed_orientation_euler = transformed_orientation.as_euler('xyz', degrees=True)
    transformed_orientation_quat = transformed_orientation.as_quat()

    return transformed_point, transformed_orientation_euler, transformed_orientation_quat
