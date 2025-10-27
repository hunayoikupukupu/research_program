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

    def solve_hand_eye_calibration(self, T_arm_from_robot_list, T_sensor_from_robot_list):
        """
        複数の T_arm_from_robot (アーム姿勢) と T_sensor_from_robot (センサー姿勢) のペアから、
        T_arm_from_robot = T_sensor_from_robot @ X となる最適な T_arm_from_sensor (X) を推定する。

        Args:
            T_arm_list (list): アーム姿勢 (T_arm_from_robot) の 4x4 行列のリスト
            T_sensor_list (list): センサー姿勢 (T_sensor_from_robot) の 4x4 行列のリスト
        
        Returns:
            np.ndarray: 推定された T_arm_from_sensor (X) の 4x4 行列
        """
        n = len(T_arm_from_robot_list)
        if n == 0:
            raise ValueError("キャリブレーションデータが空です。")
        
        R_arm_from_robot_list = []
        t_arm_from_robot_list = []
        R_sensor_from_robot_list = []
        t_sensor_from_robot_list = []
        
        # データを回転行列(R)と並進ベクトル(t)に分離
        for T_arm_from_robot, T_sensor_from_robot in zip(T_arm_from_robot_list, T_sensor_from_robot_list):
            T_arm_from_robot_transform = Transform.from_matrix(T_arm_from_robot)
            T_sensor_from_robot_transform = Transform.from_matrix(T_sensor_from_robot)
            R_arm_from_robot_list.append(T_arm_from_robot_transform.R)
            t_arm_from_robot_list.append(T_arm_from_robot_transform.t)
            R_sensor_from_robot_list.append(T_sensor_from_robot_transform.R)
            t_sensor_from_robot_list.append(T_sensor_from_robot_transform.t)

        # --- 1. 回転 R_arm_from_sensor の推定 ---
        # R_arm = R_sensor @ R_X -> R_X = R_sensor.T @ R_arm
        # 全ての R_X_i の「平均」をSVDで求める
        
        # R_X (求めたい回転) の候補の合計
        R_X_sum = np.zeros((3, 3)) 
        for R_sensor_from_robot, R_arm_from_robot in zip(R_sensor_from_robot_list, R_arm_from_robot_list):
            R_X_i = R_sensor_from_robot.T @ R_arm_from_robot
            R_X_sum += R_X_i
        
        try:
            U, S, Vt = np.linalg.svd(R_X_sum)
        except np.linalg.LinAlgError as e:
            print(f"SVD計算エラー: {e}")
            return None

        R_arm_from_sensor = U @ Vt

        # det(R) = 1 を保証 (右手座標系)
        if np.linalg.det(R_arm_from_sensor) < 0:
            Vt_copy = Vt.copy()
            Vt_copy[-1, :] *= -1 # Vの最後の行の符号を反転
            R_arm_from_sensor = U @ Vt_copy
        
        # --- 2. 並進 t_arm_from_sensor の推定 ---
        # t_arm = R_sensor @ t_X + t_sensor
        # -> R_sensor @ t_X = t_arm - t_sensor
        # これを全てのiについてスタックし、(3n x 3) @ (3 x 1) = (3n x 1) の形にする
        
        # R_sensor を縦に積んだ行列 (3n, 3)
        R_sensor_stack = np.vstack(R_sensor_from_robot_list) 
        
        t_diff_list = []
        for t_sensor_from_robot, t_arm_from_robot in zip(t_sensor_from_robot_list, t_arm_from_robot_list):
            t_diff_list.append(t_arm_from_robot - t_sensor_from_robot)

        # (t_arm - t_sensor) を縦に積んだベクトル (3n,)
        t_diff_stack = np.concatenate(t_diff_list) 
        
        # 最小二乗法で t_X (t_arm_from_sensor) を解く
        try:
            # t_arm_from_sensor が t_X に相当
            t_arm_from_sensor, _, _, _ = np.linalg.lstsq(R_sensor_stack, t_diff_stack, rcond=None)
        except np.linalg.LinAlgError as e:
            print(f"最小二乗法の計算に失敗しました: {e}")
            return None

        # --- 3. T_arm_from_sensor の組み立て ---
        T_arm_from_sensor = Transform(R_arm_from_sensor, t_arm_from_sensor).matrix
        
        return T_arm_from_sensor

    def run(self):
        """
        キャリブレーションを実行するメインメソッド。
        
        Returns:
            np.ndarray: 推定された T_arm_from_sensor の 4x4 行列
        """
        try:
            print("1. CSVからデータを読み込み、座標変換を実行しています...")
            # T_arm_from_robot_list = T_arm_from_robot のリスト
            # T_sensor_from_robot_list = T_sensor_from_robot のリスト
            T_arm_from_robot_list, T_sensor_from_robot_list = self.load_and_prepare_data()
            print(f"   {len(T_arm_from_robot_list)} 点のデータを読み込みました。")

            if len(T_arm_from_robot_list) < 3:
                print("警告: データ点数が少なすぎます。最低3点（非共線）を推奨します。")

            print("2. T_arm_from_sensor を最小二乗法で推定しています...")
            T_arm_from_sensor = self.solve_hand_eye_calibration(T_arm_from_robot_list, T_sensor_from_robot_list)

            if T_arm_from_sensor is not None:
                print("3. 推定が完了しました。")
            
            return T_arm_from_sensor
            
        except FileNotFoundError:
            print(f"エラー: CSVファイルが見つかりません: {self.csv_path}")
            return None
        except Exception as e:
            print(f"キャリブレーション中に予期せぬエラーが発生しました: {e}")
            return None