# ハンドアイキャリブレーションで[T_arm_from_sensor]を求めるための関数群

# プログラムの流れ
# 1. 引数としてhandeye_calibration用csvファイルのパスを受け取る
# 2. csvファイルを読み込み点群データを作成する([T_sensor_from_aurora],[T_arm_from_robot])
# 3. 点群データを4次元配列に整形する([T_sensor_from_aurora],[T_arm_from_robot])
# 4. ワールドキャリブレーションで求めた[T_aurora_from_robot]を使って、[T_sensor_from_aurora]=>[T_sensor_from_robot]を計算する
# 5. 複数点において対応する[T_sensor_from_robot][T_arm_from_robot]を引数としてhandeye_calibration関数に渡し、[T_arm_from_sensor]を推定する
#    計算式：T_arm_from_robot = T_sensor_from_robot @ T_arm_from_sensor => T_arm_from_sensor = inv(T_sensor_from_robot) @ T_arm_from_robot
# 6. 返り値として[T_arm_from_sensor]を返す

import numpy as np
from .transformation_utils import Transform, load_csv_data

class HandEyeCalibration:
    def __init__(self, csv_path, T_aurora_from_robot):
        self.csv_path = csv_path
        self.T_aurora_from_robot = T_aurora_from_robot
        self.T_aurora_from_robot_transform = Transform.from_matrix(T_aurora_from_robot)
    
    def load_and_prepare_data(self):
        """
        CSVからデータを読み込み、センサーの姿勢をロボット座標系に変換する。

        Returns:
            (list, list): T_arm_from_robotのリスト, T_sensor_from_robotのリスト
        """
        T_arm_from_robot_list, T_sensor_from_aurora_list = load_csv_data(self.csv_path)

        T_sensor_from_robot_list = []
        for T_sensor_from_aurora in T_sensor_from_aurora_list:
            T_sensor_from_aurora_transform = Transform.from_matrix(T_sensor_from_aurora)
            T_sensor_from_robot = self.T_aurora_from_robot_transform @ T_sensor_from_aurora_transform
            T_sensor_from_robot_list.append(T_sensor_from_robot.matrix)

        return T_arm_from_robot_list, T_sensor_from_robot_list

    def handeye_calibration(self, R_arms, R_sensors):
        """
        アーム姿勢とセンサー姿勢から変換行列を推定する
        R_arms: アーム回転行列のリスト
        R_sensors: センサー回転行列のリスト
        戻り値: 変換回転行列
        """
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