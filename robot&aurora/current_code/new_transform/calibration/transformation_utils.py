# 同次変換、逆行列、SVDなどのユーティリティ関数

import numpy as np

def find_transformation(aurora_points, robot_points):
    """
    T_robot_from_auroraを求める
    aurora_points: sensor_from_auroraの点群
    robot_points: arm_from_robotの点群
    戻り値: 回転行列R, 平行移動ベクトルt (P1 = R * P2 + t)
    """
    # 重心の計算（変数名と内容を一致させる）
    centroid_robot = np.mean(robot_points, axis=0)
    centroid_aurora = np.mean(aurora_points, axis=0)

    # 重心を基準に座標をシフト
    robot_points_centered = robot_points - centroid_robot
    aurora_points_centered = aurora_points - centroid_aurora

    # 共分散行列の計算
    H = np.dot(robot_points_centered.T, aurora_points_centered)

    # 特異値分解を実行
    U, S, V = np.linalg.svd(H, full_matrices=False)

    # 回転行列 R を計算
    R = np.dot(U, V)

    # 右手系の座標系を保つためのチェック
    if np.linalg.det(R) < 0:
        V[-1, :] = -V[-1, :]
        R = np.dot(U, V)  # または np.dot(U, V.T)

    # 並行移動ベクトル t を計算
    t = centroid_robot - np.dot(R, centroid_aurora)

    return R, t