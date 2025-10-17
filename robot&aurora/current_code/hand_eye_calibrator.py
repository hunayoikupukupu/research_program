import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import cv2
from typing import List

# --- 1. ユーティリティ関数群 ---

def axis_angle_to_matrix(axis_angle_deg):
    """
    Axis-Angle (回転ベクトル) を 3x3 回転行列に変換します。
    【修正】入力は度数法 (degrees) であることを想定し、ラジアンに変換します。
    """
    # 度からラジアンに変換
    axis_angle_rad = np.deg2rad(axis_angle_deg)
    
    angle = np.linalg.norm(axis_angle_rad)
    if angle < 1e-9:
        return np.identity(3)
    # from_rotvecはラジアン単位のベクトルを期待する
    r = Rotation.from_rotvec(axis_angle_rad)
    return r.as_matrix()

def quaternion_to_matrix(quat):
    """
    クォータニオン (x, y, z, w) を 3x3 回転行列に変換します。
    """
    # scipyは (x, y, z, w) の順
    r = Rotation.from_quat(quat)
    return r.as_matrix()

def create_homogeneous_matrix(R, t):
    """
    3x3 回転行列 R と 3x1 並進ベクトル t から 4x4 同次変換行列を作成します。
    """
    H = np.identity(4)
    H[:3, :3] = R
    H[:3, 3] = t.flatten()
    return H

# --- 2. メインのキャリブレーション関数 ---

def calculate_hand_eye_calibration(csv_path):
    """
    CSVファイルからロボットとセンサーの姿勢データを読み込み、
    ハンドアイキャリブレーション (AX=XB) を実行します。
    """
    # ... (ファイルの読み込み部分は変更なし) ...
    try:
        df = pd.read_csv(csv_path)
    except FileNotFoundError:
        print(f"エラー: 指定されたファイルが見つかりません: {csv_path}")
        return None
    except Exception as e:
        print(f"エラー: ファイルの読み込み中に問題が発生しました: {e}")
        return None

    required_columns: List[str] = [
        'robot_x', 'robot_y', 'robot_z', 'robot_rx', 'robot_ry', 'robot_rz',
        'aurora_x', 'aurora_y', 'aurora_z', 'aurora_quat_x', 'aurora_quat_y', 'aurora_quat_z', 'aurora_quat_w'
    ]
    missing_cols = [col for col in required_columns if col not in df.columns]
    if missing_cols:
        print(f"エラー: CSVファイルに必要な列がありません。不足している列: {missing_cols}")
        return None
        
    if len(df) < 3:
        print("エラー: キャリブレーションには最低3つ以上の異なる姿勢データが必要です。")
        return None

    # --- 【ここからが大きな変更点】 ---
    
    # 1. 各測定点の絶対姿勢を同次変換行列(4x4)のリストとして保存
    H_robot_base_to_arm_list = []
    H_aurora_to_sensor_list = []

    for index, row in df.iterrows():
        # ロボットアームの姿勢 (robot_base -> arm)
        t_robot_arm = row[['robot_x', 'robot_y', 'robot_z']].values.astype(float)
        # 【修正】入力は度数法なので、ラジアンに変換してから行列を作成
        axis_angle_robot_deg = row[['robot_rx', 'robot_ry', 'robot_rz']].values.astype(float)
        R_robot_arm = axis_angle_to_matrix(axis_angle_robot_deg)
        H_robot_base_to_arm_list.append(create_homogeneous_matrix(R_robot_arm, t_robot_arm))

        # Auroraセンサーの姿勢 (aurora_world -> sensor)
        t_aurora_sensor = row[['aurora_x', 'aurora_y', 'aurora_z']].values.astype(float)
        quat_aurora = row[['aurora_quat_x', 'aurora_quat_y', 'aurora_quat_z', 'aurora_quat_w']].values.astype(float)
        R_aurora_sensor = quaternion_to_matrix(quat_aurora)
        H_aurora_to_sensor_list.append(create_homogeneous_matrix(R_aurora_sensor, t_aurora_sensor))

    # 2. 連続する2つの姿勢間の相対変換を計算
    R_robot_arm_motions = []
    t_robot_arm_motions = []
    R_aurora_sensor_motions = []
    t_aurora_sensor_motions = []

    for i in range(len(H_robot_base_to_arm_list) - 1):
        # ロボットアームの相対移動量 A = (H_prev)⁻¹ * H_curr
        H_prev_robot = H_robot_base_to_arm_list[i]
        H_curr_robot = H_robot_base_to_arm_list[i+1]
        H_motion_robot = np.linalg.inv(H_prev_robot) @ H_curr_robot
        
        R_robot_arm_motions.append(H_motion_robot[:3, :3])
        t_robot_arm_motions.append(H_motion_robot[:3, 3])

        # センサーの相対移動量 B = (H_prev)⁻¹ * H_curr
        H_prev_aurora = H_aurora_to_sensor_list[i]
        H_curr_aurora = H_aurora_to_sensor_list[i+1]
        H_motion_aurora = np.linalg.inv(H_prev_aurora) @ H_curr_aurora
        
        R_aurora_sensor_motions.append(H_motion_aurora[:3, :3])
        t_aurora_sensor_motions.append(H_motion_aurora[:3, 3])

    # 3. ハンドアイキャリブレーションの実行
    # AX = XB を解く
    # A: robot_arm_motions (Gripper to Base)
    # B: aurora_sensor_motions (Target to Camera)
    # X: arm to sensor (Camera to Gripper)
    # 【修正】引数の順番を正しくする
    R_arm_sensor, t_arm_sensor = cv2.calibrateHandEye(
        R_robot_arm_motions, t_robot_arm_motions,   # A: ロボットの動き
        R_aurora_sensor_motions, t_aurora_sensor_motions, # B: センサーの動き
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    if R_arm_sensor is None or t_arm_sensor is None:
        print("エラー: キャリブレーションに失敗しました。データ点数やデータのばらつきを確認してください。")
        return None

    T_arm_sensor = create_homogeneous_matrix(R_arm_sensor, t_arm_sensor)
    return T_arm_sensor


# --- 3. プログラムの実行 ---
if __name__ == '__main__':
    
    # ▼▼▼【要設定】▼▼▼
    # データ生成プログラムで出力されたCSVファイルのパスを指定してください
    csv_file_path = "robot&aurora\current_code\calibration_data\pose_R30-120--45_T0-0-0_SEN-R-60--80-150_SEN-T10-20--30_n0_qn0.csv"
    # ▲▲▲【設定はここまで】▲▲▲

    print(f"指定されたCSVファイルを読み込みます: {csv_file_path}")
    print("-" * 50)

    # メイン関数の実行
    armTsensor = calculate_hand_eye_calibration(csv_file_path)

    # 結果の表示
    if armTsensor is not None:
        np.set_printoptions(precision=4, suppress=True)
        print("計算完了: アーム先端からセンサーへの変換行列 (${}^{arm}T_{sensor}$)")
        print(armTsensor)
        
        # 結果の解釈
        rotation_matrix = armTsensor[:3, :3]
        translation_vector = armTsensor[:3, 3]
        r = Rotation.from_matrix(rotation_matrix)
        # オイラー角は xyz 固定軸回転 (roll, pitch, yaw)
        euler_angles = r.as_euler('xyz', degrees=True)
        
        print("\n--- 結果の解釈 ---")
        print(f"並進ベクトル (x, y, z) [mm]: {translation_vector}")
        print(f"オイラー角 (roll, pitch, yaw) [deg]: {euler_angles}")

    else:
        print("\nキャリブレーション処理は正常に完了しませんでした。")