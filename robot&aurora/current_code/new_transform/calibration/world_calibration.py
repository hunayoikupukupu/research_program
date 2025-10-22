# ワールドキャリブレーションで[T_aurora_from_robot]を求めるための関数群

# プログラムの流れ
# 1. 引数としてworld_calibration用csvファイルのパスを受け取る
# 2. csvファイルを読み込み点群データを作成する
# 3. 点群データを4次元配列に整形する([T_sensor_from_aurora],[T_arm_from_robot])
# 4. 点群データ([T_sensor_from_aurora],[T_arm_from_robot])をworld_calibration関数に渡し、[T_aurora_from_robot]を推定する
# 5. 返り値として[T_aurora_from_robot]を返す

import numpy as np
from .transformation_utils import Transform, load_csv_data

class WorldCalibration:
    def __init__(self, csv_path: str):
        self.csv_path = csv_path

    def load_data_to_points(self):
        """
        CSVファイルから点群データを読み込む
        戻り値: sensor_from_auroraの点群 (N, 3), arm_from_robotの点群 (N, 3)
        """
        T_arm_from_robot_list, T_sensor_from_aurora_list = load_csv_data(self.csv_path)
        aurora_points = []
        robot_points = []

        for T_arm_from_robot, T_sensor_from_aurora in zip(T_arm_from_robot_list, T_sensor_from_aurora_list):
            T_arm_from_robot_transform = Transform.from_matrix(T_arm_from_robot)
            T_sensor_from_aurora_transform = Transform.from_matrix(T_sensor_from_aurora)
            aurora_points.append(T_sensor_from_aurora_transform.t)
            robot_points.append(T_arm_from_robot_transform.t)

        return np.array(aurora_points), np.array(robot_points)


    def compute_transform(self, aurora_points, robot_points):
        """
        T_aurora_from_robot（4x4の同次変換行列）を求める
        aurora_points: sensor_from_auroraの点群 (N, 3)
        robot_points: arm_from_robotの点群 (N, 3)
        戻り値: 4x4の同次変換行列 T (P_robot = T * P_aurora)
        """
        # 重心の計算
        centroid_robot = np.mean(robot_points, axis=0)
        centroid_aurora = np.mean(aurora_points, axis=0)

        # 重心を基準に座標をシフト
        robot_points_centered = robot_points - centroid_robot
        aurora_points_centered = aurora_points - centroid_aurora

        # 共分散行列の計算
        # H = (Q')^T * (P')  (ここで Q' = robot_points_centered, P' = aurora_points_centered)
        H = np.dot(robot_points_centered.T, aurora_points_centered)

        # 特異値分解を実行
        # np.linalg.svd は U, S, Vh (V transpose) を返す
        U, S, Vh = np.linalg.svd(H, full_matrices=False)

        # 回転行列 R を計算 (R = U * Vh)
        R = np.dot(U, Vh)

        # 右手系の座標系を保つためのチェック (リフレクションの防止)
        if np.linalg.det(R) < 0:
            # Vh の最後の行の符号を反転
            Vh[-1, :] = -Vh[-1, :]
            # R を再計算
            R = np.dot(U, Vh)

        # 並行移動ベクトル t を計算 (t = centroid_robot - R * centroid_aurora)
        t = centroid_robot - np.dot(R, centroid_aurora)

        # 4x4 の同次変換行列を作成
        T = Transform(R, t).matrix

        return T

    def run(self):
        """
        ワールドキャリブレーションを実行して T_aurora_from_robot を取得
        戻り値: 4x4の同次変換行列 T_aurora_from_robot
        """
        aurora_points, robot_points = self.load_data_to_points()
        T_aurora_from_robot = self.compute_transform(aurora_points, robot_points)
        return T_aurora_from_robot