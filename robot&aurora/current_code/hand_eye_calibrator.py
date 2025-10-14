import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import cv2
from typing import List

# --- 1. ユーティリティ関数群 ---

def axis_angle_to_matrix(axis_angle):
    """
    Axis-Angle (回転ベクトル) を 3x3 回転行列に変換します。
    """
    angle = np.linalg.norm(axis_angle)
    if angle < 1e-9:
        return np.identity(3)
    r = Rotation.from_rotvec(axis_angle)
    return r.as_matrix()

def quaternion_to_matrix(quat):
    """
    クォータニオン (x, y, z, w) を 3x3 回転行列に変換します。
    """
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
    CSVファイルに計算不要な列（例: aurora_quality）が含まれていても問題ありません。

    Args:
        csv_path (str): データが格納されたCSVファイルのパス。

    Returns:
        np.ndarray | None: 計算された 4x4 の同次変換行列 (${}^{arm}T_{sensor}$)。
                          失敗した場合は None を返します。
    """
    # 計算に必要な列名を定義
    required_columns: List[str] = [
        'robot_x', 'robot_y', 'robot_z',
        'robot_rx', 'robot_ry', 'robot_rz',
        'aurora_x', 'aurora_y', 'aurora_z',
        'aurora_quat_x', 'aurora_quat_y', 'aurora_quat_z', 'aurora_quat_w'
    ]

    # CSVファイルの読み込み
    try:
        df = pd.read_csv(csv_path)
    except FileNotFoundError:
        print(f"エラー: 指定されたファイルが見つかりません: {csv_path}")
        return None
    except Exception as e:
        print(f"エラー: ファイルの読み込み中に問題が発生しました: {e}")
        return None
        
    # 【追加】必須列がすべて存在するかチェック
    missing_cols = [col for col in required_columns if col not in df.columns]
    if missing_cols:
        print(f"エラー: CSVファイルに必要な列がありません。不足している列: {missing_cols}")
        return None

    # OpenCVの関数に渡すためのリストを準備
    R_robot_arm_list = []
    t_robot_arm_list = []
    R_aurora_sensor_list = []
    t_aurora_sensor_list = []

    # データフレームの各行を処理
    for index, row in df.iterrows():
        # 【変更なし】必要な列名だけを指定してデータを抽出するため、
        # 'aurora_quality'などの余分な列は自動的に無視されます。
        
        # ロボットアームの姿勢 (robot -> arm)
        t_robot_arm = row[['robot_x', 'robot_y', 'robot_z']].values.astype(float)
        axis_angle_robot = row[['robot_rx', 'robot_ry', 'robot_rz']].values.astype(float)
        R_robot_arm = axis_angle_to_matrix(axis_angle_robot)
        
        R_robot_arm_list.append(R_robot_arm)
        t_robot_arm_list.append(t_robot_arm)

        # Auroraセンサーの姿勢 (aurora -> sensor)
        t_aurora_sensor = row[['aurora_x', 'aurora_y', 'aurora_z']].values.astype(float)
        quat_aurora = row[['aurora_quat_x', 'aurora_quat_y', 'aurora_quat_z', 'aurora_quat_w']].values.astype(float)
        R_aurora_sensor = quaternion_to_matrix(quat_aurora)

        R_aurora_sensor_list.append(R_aurora_sensor)
        t_aurora_sensor_list.append(t_aurora_sensor)

    if len(R_robot_arm_list) < 3:
        print("エラー: キャリブレーションには最低3つ以上の異なる姿勢データが必要です。")
        return None

    # ハンドアイキャリブレーションの実行
    R_arm_sensor, t_arm_sensor = cv2.calibrateHandEye(
        R_aurora_sensor_list, t_aurora_sensor_list,
        R_robot_arm_list, t_robot_arm_list,
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
    # 測定データが保存されているCSVファイルのパスをここに指定してください。
    csv_file_path = "robot&aurora/current_code/calibration_data/aurora_robot_pose_log_6_6_6.csv"
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
        euler_angles = r.as_euler('xyz', degrees=True)
        
        print("\n--- 結果の解釈 ---")
        print(f"並進ベクトル (x, y, z) [m]: {translation_vector}")
        print(f"オイラー角 (roll, pitch, yaw) [deg]: {euler_angles}")
    else:
        print("\nキャリブレーション処理は正常に完了しませんでした。")