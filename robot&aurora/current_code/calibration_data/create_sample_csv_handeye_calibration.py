import numpy as np
from scipy.spatial.transform import Rotation as R
import csv
import os

def generate_synthetic_data(
    rotation_euler_deg_aur2rob, 
    translation_vector_aur2rob, 
    rotation_euler_deg_sen2arm,
    translation_vector_sen2arm,
    x_range, 
    y_range, 
    z_range, 
    rx_range,
    ry_range,
    rz_range,
    step_size=20, 
    rot_step_size=30,
    output_file="robot&aurora/current_code/calibration_data/aurora_robot_pose_log.csv",
    add_noise=0,
    add_quaternion_noise=0
):
    """
    ロボット座標系とAurora座標系の変換関係に基づいて合成データを生成する

    :param rotation_euler_deg_aur2rob: 回転角度（オイラー角、度数法）roll, pitch, yaw（固定軸回転zyx）
    :param translation_vector_aur2rob: 並進ベクトル [tx, ty, tz]
    :param rotation_euler_deg_sen2arm: センサー座標系からアーム座標系への回転角度（オイラー角、度数法）[roll, pitch, yaw]
    :param translation_vector_sen2arm: センサー座標系からアーム座標系への並進ベクトル [tx, ty, tz]
    :param x_range: X座標の範囲 (開始値, 終了値)
    :param y_range: Y座標の範囲 (開始値, 終了値)
    :param z_range: Z座標の範囲 (開始値, 終了値)
    :param rx_range: ロボットアームのX軸周り回転の範囲 (度数法)
    :param ry_range: ロボットアームのY軸周り回転の範囲 (度数法)
    :param rz_range: ロボットアームのZ軸周り回転の範囲 (度数法)
    :param step_size: サンプリング間隔（mm）
    :param rot_step_size: 回転のサンプリング間隔 (度)
    :param output_file: 出力CSVファイル名
    :param add_noise: Aurora座標に追加するガウスノイズの標準偏差（mm）
    :param add_quaternion_noise: クォータニオンに追加するガウスノイズの標準偏差
    :return: 生成したデータポイント
    """

    # Aurora -> Robot の変換行列
    euler_aur2rob_ypr = [rotation_euler_deg_aur2rob[2], rotation_euler_deg_aur2rob[1], rotation_euler_deg_aur2rob[0]]
    rotation_aur2rob = R.from_euler('zyx', euler_aur2rob_ypr, degrees=True)
    R_matrix_aur2rob = rotation_aur2rob.as_matrix()
    R_matrix_aur2rob_rounded = np.round(R_matrix_aur2rob, 5)
    print(f"R_matrix_aur2rob:\n{R_matrix_aur2rob_rounded}")
    print(f"rotation_euler_deg_aur2rob: {rotation_euler_deg_aur2rob}")

    # Sensor -> Arm の回転行列
    euler_sen2arm_ypr = [rotation_euler_deg_sen2arm[2], rotation_euler_deg_sen2arm[1], rotation_euler_deg_sen2arm[0]]
    rotation_sen2arm = R.from_euler('zyx', euler_sen2arm_ypr, degrees=True)
    R_matrix_sen2arm = rotation_sen2arm.as_matrix()
    R_matrix_sen2arm_rounded = np.round(R_matrix_sen2arm, 5)
    print(f"R_matrix_sen2arm:\n{R_matrix_sen2arm_rounded}")
    print(f"rotation_euler_deg_sen2arm: {rotation_euler_deg_sen2arm}")
    print(f"translation_vector_sen2arm: {translation_vector_sen2arm}")

    # サンプリング点を生成
    x_points = np.arange(x_range[0], x_range[1] + step_size/2, step_size)
    y_points = np.arange(y_range[0], y_range[1] + step_size/2, step_size)
    z_points = np.arange(z_range[0], z_range[1] + step_size/2, step_size)
    
    # 回転のサンプリング点を生成
    rx_points = np.arange(rx_range[0], rx_range[1] + rot_step_size/2, rot_step_size)
    ry_points = np.arange(ry_range[0], ry_range[1] + rot_step_size/2, rot_step_size)
    rz_points = np.arange(rz_range[0], rz_range[1] + rot_step_size/2, rot_step_size)

    # 出力ディレクトリの確保
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    data_points = []
    
    # 全ての点の組み合わせに対してAurora座標を計算
    for x in x_points:
        for y in y_points:
            for z in z_points:
                for rx in rx_points:
                    for ry in ry_points:
                        for rz in rz_points:
                            robot_arm_position = np.array([x, y, z])
                            robot_arm_R_vector_deg = np.array([rx, ry, rz])
                            
                            # ロボットアームの回転
                            rotation_arm_from_robot = R.from_rotvec(robot_arm_R_vector_deg, degrees=True)
                            R_matrix_arm_from_robot = rotation_arm_from_robot.as_matrix()

                            # --- 姿勢の計算 ---
                            # ロボット座標系におけるセンサー姿勢をアーム姿勢に変換
                            R_matrix_sensor_from_robot = R_matrix_sen2arm.T @ R_matrix_arm_from_robot
                            R_matrix_sensor_from_aurora = R_matrix_aur2rob.T @ R_matrix_sensor_from_robot
                            rotation_sensor_from_aurora = R.from_matrix(R_matrix_sensor_from_aurora)
                            quaternion_sensor_from_aurora = rotation_sensor_from_aurora.as_quat()

                            # --- 位置の計算 ---
                            # センサーの並進を考慮したロボット座標系でのセンサー位置
                            # sensor_pos_in_robot = arm_pos_in_robot + R_arm_from_robot * T_sen2arm
                            sensor_position_in_robot_frame = robot_arm_position + R_matrix_arm_from_robot @ translation_vector_sen2arm
                            # ロボット座標からAurora座標への変換
                            aurora_point = R_matrix_aur2rob.T @ (sensor_position_in_robot_frame - translation_vector_aur2rob)

                            # 位置座標にノイズの追加
                            if add_noise > 0:
                                aurora_point += np.random.normal(0, add_noise, 3)
                            
                            noisy_quaternion = quaternion_sensor_from_aurora.copy()
                            
                            # クォータニオンにノイズの追加
                            if add_quaternion_noise > 0:
                                quaternion_noise = np.random.normal(0, add_quaternion_noise, 4)
                                noisy_quaternion += quaternion_noise
                                quaternion_norm = np.linalg.norm(noisy_quaternion)
                                if quaternion_norm > 0:
                                    noisy_quaternion /= quaternion_norm
                            
                            # データポイントを追加
                            data_points.append([
                                round(x, 5), round(y, 5), round(z, 5),
                                round(rx, 5), round(ry, 5), round(rz, 5),
                                round(aurora_point[0], 5), round(aurora_point[1], 5), round(aurora_point[2], 5),
                                round(noisy_quaternion[0], 5), round(noisy_quaternion[1], 5), round(noisy_quaternion[2], 5), round(noisy_quaternion[3], 5),
                                0.1
                            ])
    
    # CSVファイルに書き込み
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['robot_x', 'robot_y', 'robot_z', 'robot_rx', 'robot_ry', 'robot_rz', 'aurora_x', 'aurora_y', 'aurora_z', 'aurora_quat_x', 'aurora_quat_y', 'aurora_quat_z', 'aurora_quat_w', 'aurora_quality'])
        writer.writerows(data_points)
    
    print(f"生成したデータポイント数: {len(data_points)}")
    print(f"データを {output_file} に保存しました")
    print(f"位置ノイズ標準偏差: {add_noise} mm")
    print(f"クォータニオンノイズ標準偏差: {add_quaternion_noise}")
    
    return data_points

def main(
    x_range,
    y_range,
    z_range,
    rx_range,
    ry_range,
    rz_range,
    rotation_euler_deg_aur2rob=None,
    translation_vector_aur2rob=None, 
    rotation_euler_deg_sen2arm=None,
    translation_vector_sen2arm=None,
    step_size=None,
    rot_step_size=None,
    output_file="robot&aurora/current_code/calibration_data/aurora_robot_pose_log.csv",
    add_noise=0,
    add_quaternion_noise=0
):
    """
    メイン処理を実行する関数
    """
    # デフォルトパラメータの設定
    if rotation_euler_deg_aur2rob is None:
        rotation_euler_deg_aur2rob = [0, 0, 90]
    if translation_vector_aur2rob is None:
        translation_vector_aur2rob = [0, 0, -100]
    if rotation_euler_deg_sen2arm is None:
        rotation_euler_deg_sen2arm = [0, 0, 0]
    if translation_vector_sen2arm is None:
        translation_vector_sen2arm = [0, 0, 0]
    if step_size is None:
        step_size = 20
    if rot_step_size is None:
        rot_step_size = 30
        
    return generate_synthetic_data(
        rotation_euler_deg_aur2rob=rotation_euler_deg_aur2rob,
        translation_vector_aur2rob=translation_vector_aur2rob,
        rotation_euler_deg_sen2arm=rotation_euler_deg_sen2arm,
        translation_vector_sen2arm=translation_vector_sen2arm,
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        rx_range=rx_range,
        ry_range=ry_range,
        rz_range=rz_range,
        step_size=step_size,
        rot_step_size=rot_step_size,
        output_file=output_file,
        add_noise=add_noise,
        add_quaternion_noise=add_quaternion_noise
    )

def generate_output_filename(rotation_euler_deg_aur2rob, translation_vector_aur2rob, 
                             rotation_euler_deg_sen2arm, translation_vector_sen2arm, 
                             add_noise, add_quaternion_noise=0):
    """
    パラメータから動的にファイル名を生成する関数
    """
    rot_str = "-".join(map(str, rotation_euler_deg_aur2rob))
    trans_str = "-".join(map(str, translation_vector_aur2rob))
    sen_rot_str = "-".join(map(str, rotation_euler_deg_sen2arm))
    sen_trans_str = "-".join(map(str, translation_vector_sen2arm))
    
    filename = (
        f"robot&aurora/current_code/calibration_data/"
        f"pose_R{rot_str}_T{trans_str}_"
        f"SEN-R{sen_rot_str}_SEN-T{sen_trans_str}_"
        f"n{add_noise}_qn{add_quaternion_noise}.csv"
    )
    
    return filename

if __name__ == "__main__":
    # パラメータを一か所で設定
    rotation_euler_deg_aur2rob = [10, -40, 60]  # Aurora座標系からロボット座標系へのオイラー角（度）[roll, pitch, yaw]
    translation_vector_aur2rob = [20, 10, -150]    # Aurora座標系からロボット座標系への並進ベクトル（mm）[tx, ty, tz]
    
    # センサーとアームの関係性を定義
    rotation_euler_deg_sen2arm = [-40, 170, 90] # センサー座標系からアーム座標系へのオイラー角（度）[roll, pitch, yaw]
    translation_vector_sen2arm = [0, 0, 0] # センサー座標系からアーム座標系への並進ベクトル（mm）[tx, ty, tz]
    
    add_noise = 0              # 位置ノイズの標準偏差（mm）
    add_quaternion_noise = 0   # クォータニオンノイズの標準偏差
    
    # 動的にファイル名を生成
    output_file = generate_output_filename(
        rotation_euler_deg_aur2rob,
        translation_vector_aur2rob,
        rotation_euler_deg_sen2arm,
        translation_vector_sen2arm,
        add_noise,
        add_quaternion_noise
    )
    
    print(f"Generated filename: {output_file}")
    
    # main関数の呼び出し
    result = main(
        x_range=(75, 175),      # X座標の範囲（mm）
        y_range=(-50, 50),      # Y座標の範囲（mm）
        z_range=(-350, -250),   # Z座標の範囲（mm）
        rx_range=(0, 30),         # ロボットアームのrx範囲（度）
        ry_range=(0, 60),         # ロボットアームのry範囲（度）
        rz_range=(0, 90),         # ロボットアームのrz範囲（度）
        rotation_euler_deg_aur2rob=rotation_euler_deg_aur2rob,
        translation_vector_aur2rob=translation_vector_aur2rob,
        rotation_euler_deg_sen2arm=rotation_euler_deg_sen2arm,
        translation_vector_sen2arm=translation_vector_sen2arm,
        step_size=50,             # 位置のステップサイズ（mm）
        rot_step_size=30,         # 回転のステップサイズ（度）
        output_file=output_file,
        add_noise=add_noise,
        add_quaternion_noise=add_quaternion_noise
    )