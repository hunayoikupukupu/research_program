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
    arm_pose_rot = R.from_euler('xyz', np.deg2rad(arm_pose_euler))  # 順序はロボットの定義に合わせて調整
    arm_pose_matrix = arm_pose_rot.as_matrix()
    
    # ステップ3: R2を計算 (センサー座標系からアーム座標系への変換)
    # R2 = arm_pose_matrix * (sensor_pose_robot_matrix)^(-1)
    # つまり、R2 = アーム姿勢行列 * センサー姿勢行列の逆行列
    R2 = np.dot(arm_pose_matrix, np.linalg.inv(sensor_pose_robot_matrix))
    
    return R2

def verify_r2_calculation(R2, ideal_sensor_pose_aurora_quat, T1, R1, ideal_arm_position=None):
    """
    計算されたR2を使用して、理想的なセンサー位置・姿勢から
    理想的なアーム姿勢を計算して検証する関数
    
    Parameters:
    -----------
    R2 : array-like
        計算されたセンサー⇒アームの回転行列
    ideal_sensor_pose_aurora_quat : array-like
        理想的なセンサーの姿勢（クォータニオン [x, y, z, w]）
    T1 : array-like
        Aurora座標系からロボット座標系への並進ベクトル
    R1 : array-like
        Aurora座標系からロボット座標系への回転行列
    ideal_arm_position : array-like, optional
        理想的なアーム位置（設定されていない場合は理想センサー位置を使用）
    
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
    ideal_arm_pose_euler_rad = ideal_arm_pose_rot.as_euler('xyz')  # 順序はロボットの定義に合わせて調整
    ideal_arm_pose_euler = np.rad2deg(ideal_arm_pose_euler_rad)  # ラジアンから度に変換
    
    # 理想アーム位置を計算（デフォルトでは理想センサー位置と同じ）
    if ideal_arm_position is None:
        # センサー位置をロボット座標系に変換
        ideal_sensor_position_robot = np.dot(R1, ideal_sensor_pose_aurora_rot.as_rotvec()) + T1
        ideal_arm_position = ideal_sensor_position_robot
    
    return ideal_arm_pose_euler, ideal_arm_position

# 使用例
if __name__ == "__main__":
    # 例のデータ（実際の使用時には実測値を使用）
    # AuroraからのセンサーのクォータニオンとT1、R1は既に取得済みと仮定
    sensor_pose_aurora_quat = np.array([0.1, 0.2, 0.3, 0.9])  # [x, y, z, w]
    arm_pose_euler = np.array([5.7, 11.5, 17.2])  # [roll, pitch, yaw] (度)
    
    # 既に計算済みのR1（Aurora⇒ロボット）
    R1 = np.eye(3)  # 例として単位行列を使用
    
    # R2の計算
    R2 = compute_rotation_matrix_R2(sensor_pose_aurora_quat, arm_pose_euler, R1)
    print("R2 (センサー⇒アーム):")
    print(R2)
    
    # 検証: 理想センサー姿勢から理想アーム姿勢を計算
    ideal_sensor_pose_aurora_quat = np.array([0.0, 0.0, 0.0, 1.0])  # 理想クォータニオン [x, y, z, w]
    T1 = np.array([10.0, 20.0, 30.0])  # 例として
    
    ideal_arm_pose_euler, ideal_arm_position = verify_r2_calculation(R2, ideal_sensor_pose_aurora_quat, T1, R1)
    
    print("\n理想アーム姿勢 (オイラー角, 度):")
    print(ideal_arm_pose_euler)
    print("\n理想アーム位置:")
    print(ideal_arm_position)