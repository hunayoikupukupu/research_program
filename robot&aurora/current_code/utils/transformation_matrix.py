import numpy as np

def find_transformation(P2, P1):
    """
    座標系2が座標系1に対する変換パラメータを求める
    P2: 座標系2での点群
    P1: 座標系1での点群
    戻り値: 回転行列R, 平行移動ベクトルt (P1 = R * P2 + t)
    """
    # 重心の計算（変数名と内容を一致させる）
    centroid_P1 = np.mean(P1, axis=0)
    centroid_P2 = np.mean(P2, axis=0)

    # 重心を基準に座標をシフト
    P1_centered = P1 - centroid_P1
    P2_centered = P2 - centroid_P2

    # 共分散行列の計算
    H = np.dot(P1_centered.T, P2_centered)

    # 特異値分解を実行
    U, S, V = np.linalg.svd(H, full_matrices=False)

    # 回転行列 R を計算
    R = np.dot(U, V)

    # 右手系の座標系を保つためのチェック
    if np.linalg.det(R) < 0:
        V[-1, :] = -V[-1, :]
        R = np.dot(U, V)  # または np.dot(U, V.T)

    # 並行移動ベクトル t を計算
    t = centroid_P1 - np.dot(R, centroid_P2)

    return R, t

def estimate_transform_matrix(R_arms, R_sensors):
    n = len(R_arms)
    
    # 各観測に対して R_sensor_i * R_arm_i^T を計算
    M_sum = np.zeros((3, 3))
    for i in range(n):
        M_i = np.dot(R_arms[i], R_sensors[i].T)
        M_sum += M_i
    
    # 平均を取る
    M_avg = M_sum / n
    
    # SVD分解で最も近い回転行列を求める
    U, S, Vt = np.linalg.svd(M_avg)
    R_transform = np.dot(U, Vt)
    
    # 右手系を保証（det(R) = 1）
    if np.linalg.det(R_transform) < 0:
        Vt[-1, :] *= -1
        R_transform = np.dot(U, Vt)
    
    return R_transform