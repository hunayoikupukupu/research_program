import numpy as np
from scipy.spatial.transform import Rotation as R

def compute_rotation_matrix_R2(sensor_pose_aurora_quat, arm_pose_euler, R1):
    """
    センサー⇒アームの回転行列R2を計算する関数
    
    Parameters:
    -----------
    sensor_pose_aurora_quat : array-like
        Auroraから得られたセンサーの姿勢情報（クォータニオン [x, y, z, w]）
    arm_pose_euler : array-like
        ロボットアームの姿勢情報（オイラー角 [roll, pitch, yaw]）（度）
    R1 : array-like
        Aurora座標系からロボット座標系への回転行列（3x3）
    
    Returns:
    --------
    R2 : numpy.ndarray
        センサー座標系からアーム座標系への回転行列（3x3）
    """
    # ステップ1: Auroraから得られたセンサーの姿勢（クォータニオン）をロボット座標系に変換
    # クォータニオンから回転行列に変換
    sensor_pose_aurora_rot = R.from_quat(sensor_pose_aurora_quat)
    sensor_pose_aurora_matrix = sensor_pose_aurora_rot.as_matrix()
    
    # Aurora座標系のセンサー姿勢をロボット座標系に変換
    # R1 * sensor_pose_aurora_matrix
    sensor_pose_robot_matrix = np.dot(R1, sensor_pose_aurora_matrix)
    sensor_pose_robot_rot = R.from_matrix(sensor_pose_robot_matrix)
    
    # ステップ2: ロボットアームの姿勢（オイラー角、度）を回転行列に変換
    # 度からラジアンに変換してから回転行列を作成
    # 固定軸回転（extrinsic rotation）を使用：'XYZ'と大文字で指定する
    arm_pose_rot = R.from_euler('XYZ', np.deg2rad(arm_pose_euler))
    arm_pose_matrix = arm_pose_rot.as_matrix()
    
    # ステップ3: R2を計算 (センサー座標系からアーム座標系への変換)
    # R2 = arm_pose_matrix * (sensor_pose_robot_matrix)^(-1)
    # つまり、R2 = アーム姿勢行列 * センサー姿勢行列の逆行列
    R2 = np.dot(arm_pose_matrix, np.linalg.inv(sensor_pose_robot_matrix))
    
    return R2

def verify_r2_calculation(R2, ideal_sensor_pose_aurora_quat, ideal_sensor_position_aurora, T1, R1):
    """
    計算されたR2を使用して、理想的なセンサー位置・姿勢から
    理想的なアーム姿勢を計算して検証する関数
    
    Parameters:
    -----------
    R2 : array-like
        計算されたセンサー⇒アームの回転行列
    ideal_sensor_pose_aurora_quat : array-like
        理想的なセンサーの姿勢（クォータニオン [x, y, z, w]）
    ideal_sensor_position_aurora : array-like
        理想的なセンサーの位置（Aurora座標系）
    T1 : array-like
        Aurora座標系からロボット座標系への並進ベクトル
    R1 : array-like
        Aurora座標系からロボット座標系への回転行列
    
    Returns:
    --------
    ideal_arm_pose_euler : numpy.ndarray
        理想的なアーム姿勢（オイラー角 [roll, pitch, yaw]）（度）
    ideal_arm_position : numpy.ndarray
        理想的なアーム位置
    """
    # 理想センサーの姿勢をクォータニオンから回転行列に変換
    ideal_sensor_pose_aurora_rot = R.from_quat(ideal_sensor_pose_aurora_quat)
    ideal_sensor_pose_aurora_matrix = ideal_sensor_pose_aurora_rot.as_matrix()
    
    # Aurora座標系の理想センサー姿勢をロボット座標系に変換
    ideal_sensor_pose_robot_matrix = np.dot(R1, ideal_sensor_pose_aurora_matrix)
    
    # R2を使って理想アーム姿勢の回転行列を計算
    ideal_arm_pose_matrix = np.dot(R2, ideal_sensor_pose_robot_matrix)
    
    # 回転行列からオイラー角に変換（ラジアンから度に変換）
    ideal_arm_pose_rot = R.from_matrix(ideal_arm_pose_matrix)
    # 固定軸回転（extrinsic rotation）を使用：'XYZ'と大文字で指定する
    ideal_arm_pose_euler_rad = ideal_arm_pose_rot.as_euler('XYZ')
    ideal_arm_pose_euler = np.rad2deg(ideal_arm_pose_euler_rad)  # ラジアンから度に変換
    
    # 理想センサー位置をAurora座標系からロボット座標系に変換
    ideal_sensor_position_robot = np.dot(R1, ideal_sensor_position_aurora) + T1
    
    # 理想アーム位置は変換後のセンサー位置と同じ（センサー原点とアーム原点は同じため）
    ideal_arm_position = ideal_sensor_position_robot
    
    return ideal_arm_pose_euler, ideal_arm_position

# 使用例
if __name__ == "__main__":
    print("===== R2計算のテスト =====")
    
    # テスト1: シンプルな直交姿勢（基本ケース）
    print("\nテスト1: 基本ケース - 直交姿勢")
    # Auroraからのセンサーのクォータニオン - 単位クォータニオン（回転なし）
    sensor_pose_aurora_quat = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w]
    # アームの姿勢 - Zを90度回転（初期位置から正面に向ける）
    arm_pose_euler = np.array([0.0, 0.0, 90.0])  # [roll, pitch, yaw] (度)
    
    # Aurora⇒ロボットの回転行列（ここでは単位行列を使用）
    R1 = np.eye(3)
    
    # R2の計算
    R2 = compute_rotation_matrix_R2(sensor_pose_aurora_quat, arm_pose_euler, R1)
    print("R2 (センサー⇒アーム):")
    print(R2)
    
    # 検証: センサー姿勢の変化に対するアーム姿勢の計算
    print("\nR2の検証:")
    # 理想センサー姿勢 - X軸周りに30度回転
    x_rotation_quat = R.from_euler('X', 30, degrees=True).as_quat()  # 固定軸表記
    ideal_sensor_pose_aurora_quat = x_rotation_quat
    print(f"理想センサー姿勢（クォータニオン）: {ideal_sensor_pose_aurora_quat}")
    
    # 理想センサー位置（Aurora座標系）
    ideal_sensor_position_aurora = np.array([50.0, 75.0, 100.0])
    
    # Aurora⇒ロボットの並進ベクトル（座標変換用）
    T1 = np.array([100.0, 200.0, 300.0])  # わかりやすい数値（mm単位と仮定）
    
    # 理想アーム姿勢の計算
    ideal_arm_pose_euler, ideal_arm_position = verify_r2_calculation(
        R2, ideal_sensor_pose_aurora_quat, ideal_sensor_position_aurora, T1, R1
    )
    
    print("\n理想アーム姿勢 (オイラー角, 度):")
    print(f"Roll: {ideal_arm_pose_euler[0]:.2f}°")
    print(f"Pitch: {ideal_arm_pose_euler[1]:.2f}°")
    print(f"Yaw: {ideal_arm_pose_euler[2]:.2f}°")
    
    print("\n理想アーム位置:")
    print(f"X: {ideal_arm_position[0]:.2f}mm")
    print(f"Y: {ideal_arm_position[1]:.2f}mm")
    print(f"Z: {ideal_arm_position[2]:.2f}mm")
    
    # テスト2: 別の姿勢
    print("\n\nテスト2: 別の姿勢設定")
    # AuroraからのセンサーのクォータニオンとT1、R1は既に取得済みと仮定
    sensor_pose_aurora_quat = R.from_euler('XYZ', [10, 20, 30], degrees=True).as_quat()  # 固定軸表記
    arm_pose_euler = np.array([45.0, 30.0, 60.0])  # [roll, pitch, yaw] (度)
    
    # Aurora⇒ロボットの回転行列（ここでは45度のZ回転を適用）
    R1 = R.from_euler('Z', 45, degrees=True).as_matrix()  # 固定軸表記
    
    # R2の計算
    R2 = compute_rotation_matrix_R2(sensor_pose_aurora_quat, arm_pose_euler, R1)
    print("R2 (センサー⇒アーム):")
    print(R2)
    
    # 検証: 理想センサー姿勢から理想アーム姿勢を計算
    # 理想センサー姿勢 - 特定の方向を向かせる
    ideal_sensor_pose_aurora_quat = R.from_euler('XYZ', [0, 45, 0], degrees=True).as_quat()  # 固定軸表記
    print(f"理想センサー姿勢（クォータニオン）: {ideal_sensor_pose_aurora_quat}")
    
    # 理想センサー位置（Aurora座標系）
    ideal_sensor_position_aurora = np.array([25.0, 50.0, 75.0])
    
    T1 = np.array([50.0, 100.0, 150.0])
    
    ideal_arm_pose_euler, ideal_arm_position = verify_r2_calculation(
        R2, ideal_sensor_pose_aurora_quat, ideal_sensor_position_aurora, T1, R1
    )
    
    print("\n理想アーム姿勢 (オイラー角, 度):")
    print(f"Roll: {ideal_arm_pose_euler[0]:.2f}°")
    print(f"Pitch: {ideal_arm_pose_euler[1]:.2f}°")
    print(f"Yaw: {ideal_arm_pose_euler[2]:.2f}°")
    
    print("\n理想アーム位置:")
    print(f"X: {ideal_arm_position[0]:.2f}mm")
    print(f"Y: {ideal_arm_position[1]:.2f}mm") 
    print(f"Z: {ideal_arm_position[2]:.2f}mm")