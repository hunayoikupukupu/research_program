import numpy as np
from scipy.spatial.transform import Rotation as R

def transform_point_and_orientation(point_A, quaternion_A, origin_B, quaternion_B):
    """
    座標系A上の点と姿勢を座標系Bへ変換する。
    :param point_A: 座標系A上の点 (x, y, z)
    :param quaternion_A: 座標系Aにおける姿勢 (クォータニオン)
    :param origin_B: 座標系Bの原点
    :param quaternion_B: 座標系Bのクォータニオン
    :return: (座標系Bにおける点Pの座標, 座標系Bにおける姿勢 (クォータニオン))
    """
    # 座標変換: クォータニオンの逆回転と並進を適用
    rotation_B_inv = R.from_quat(quaternion_B).inv()  # 座標系BからAへの逆回転
    transformed_point = rotation_B_inv.apply(point_A - origin_B)  # 並進を反映後に回転適用

    # y, z 軸を反転（座標系の定義に応じて）
    transformed_point[1] *= -1
    transformed_point[2] *= -1

    # 姿勢変換: クォータニオンを使用
    rotation_A = R.from_quat(quaternion_A)
    transformed_orientation = rotation_B_inv * rotation_A  # 座標系変換を正確に適用
    transformed_orientation_euler = transformed_orientation.as_euler('xyz', degrees=True)
    transformed_orientation_quat = transformed_orientation.as_quat()

    return transformed_point, transformed_orientation_euler, transformed_orientation_quat
