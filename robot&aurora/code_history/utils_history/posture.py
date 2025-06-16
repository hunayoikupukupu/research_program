import numpy as np
from scipy.spatial.transform import Rotation

def findQuatanion(transform_matrix, aurora_quat):
    # 回転行列からクォータニオンを取得
    transform_quat = Rotation.from_matrix(transform_matrix).as_quat()
    
    print(f"Transform Quat: {transform_quat}")

    # クォータニオンの正規化
    transform_quat = transform_quat / np.linalg.norm(transform_quat)
    aurora_quat = aurora_quat / np.linalg.norm(aurora_quat)
    
    # クォータニオンの積を計算
    robot_quat = Rotation.from_quat(aurora_quat) * Rotation.from_quat(transform_quat)
    
    # オイラー角に変換
    robot_roll, robot_pitch, robot_yaw = robot_quat.as_euler('xyz', degrees=True)
    
    # 結果を出力
    print(f"Roll: {robot_roll}, Pitch: {robot_pitch}, Yaw: {robot_yaw}")
    
    return robot_roll, robot_pitch, robot_yaw


def findQuatanion_2(transform_matrix, aurora_quat):

    rot_aurora = Rotation.from_quat(aurora_quat)
    rot_transform = Rotation.from_matrix(transform_matrix @ rot_aurora.as_matrix())

    euler_angles = rot_transform.as_euler('xyz', degrees=True)
    
    return euler_angles