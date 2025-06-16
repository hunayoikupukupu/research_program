import numpy as np

def findTransformation(P2, P1):
    # 座標系2が座標系1に対してどれだけ回転しているか(R)，どれだけ並行移動しているか(t)を求める
    """
    座標系2が座標系1に対する変換パラメータを求める
    P2: 座標系2での点群
    P1: 座標系1での点群
    戻り値: 回転行列R, 平行移動ベクトルt
    """
    # 重心の計算
    centroid_P1 = np.mean(P2, axis=0)
    centroid_P2 = np.mean(P1, axis=0)

    # 重心を基準に座標をシフト
    P1_centered = P2 - centroid_P1
    P2_centered = P1 - centroid_P2

    # 共分散行列の計算
    H = np.dot(P1_centered.T, P2_centered)

    # 特異値分解を実行
    U, S, V = np.linalg.svd(H, full_matrices=False)
    H2 = np.dot(U, np.dot(np.diag(S), V))


    # 回転行列 R を計算
    R = np.dot(V.T, U.T)

    # 並行移動ベクトル t を計算
    t = centroid_P2 - np.dot(R, centroid_P1)

    # 回転行列が右手系の座標系を保つためのチェック
    if np.linalg.det(R) < 0:
        V[-1, :] = -V[-1, :]
        R = np.dot(V.T, U.T)

    return R, t