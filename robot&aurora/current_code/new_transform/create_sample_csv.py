import numpy as np
from scipy.spatial.transform import Rotation as R
from calibration.transformation_utils import Transform

import csv
import os

def generate_synthetic_data(
    output_file,
    euler_aurora_from_robot, 
    t_aurora_from_robot, 
    euler_arm_from_sensor,
    t_arm_from_sensor,
    x_range, 
    y_range, 
    z_range, 
    rx_range,
    ry_range,
    rz_range,
    step_size, 
    rot_step_size,
    add_noise,
    add_quaternion_noise
):
    """
    ロボット座標系とAurora座標系の変換関係に基づいて合成データを生成する

    :param output_file: 出力CSVファイル名
    :param euler_aurora_from_robot: robot座標系から見たaurora座標系の回転角度（オイラー角、度数法）[roll, pitch, yaw]（固定軸回転zyx）
    :param t_aurora_from_robot: 並進ベクトル [tx, ty, tz]
    :param euler_arm_from_sensor: センサー座標系から見たアーム座標系の回転角度（オイラー角、度数法）[roll, pitch, yaw]（固定軸回転zyx）
    :param t_arm_from_sensor: センサー座標系からアーム座標系への並進ベクトル [tx, ty, tz]
    :param x_range: X座標の範囲 (開始値, 終了値)
    :param y_range: Y座標の範囲 (開始値, 終了値)
    :param z_range: Z座標の範囲 (開始値, 終了値)
    :param rx_range: ロボットアームのX軸周り回転の範囲 (度数法)
    :param ry_range: ロボットアームのY軸周り回転の範囲 (度数法)
    :param rz_range: ロボットアームのZ軸周り回転の範囲 (度数法)
    :param step_size: サンプリング間隔（mm）
    :param rot_step_size: 回転のサンプリング間隔 (度)
    :param add_noise: Aurora座標に追加するガウスノイズの標準偏差（mm）
    :param add_quaternion_noise: クォータニオンに追加するガウスノイズの標準偏差
    :return: 生成したデータポイント
    """

    # Aurora -> Robot の同次変換行列を作成
    euler_aurora_from_robot_ypr = [euler_aurora_from_robot[2], euler_aurora_from_robot[1], euler_aurora_from_robot[0]]
    R_aurora_from_robot = R.from_euler('zyx', euler_aurora_from_robot_ypr, degrees=True).as_matrix()
    R_aurora_from_robot_rounded = np.round(R_aurora_from_robot, 5)
    T_aurora_from_robot_transform = Transform(R_aurora_from_robot, t_aurora_from_robot)
    print(f"R_aurora_from_robot:\n{R_aurora_from_robot_rounded}")
    print(f"euler_aurora_from_robot: {euler_aurora_from_robot}")
    print(f"t_aurora_from_robot: {t_aurora_from_robot}")

    # Sensor -> Arm の同次変換行列を作成
    euler_arm_from_sensor_ypr = [euler_arm_from_sensor[2], euler_arm_from_sensor[1], euler_arm_from_sensor[0]]
    R_arm_from_sensor = R.from_euler('zyx', euler_arm_from_sensor_ypr, degrees=True).as_matrix()
    R_arm_from_sensor_rounded = np.round(R_arm_from_sensor, 5)
    T_arm_from_sensor_transform = Transform(R_arm_from_sensor, t_arm_from_sensor)
    print(f"R_arm_from_sensor:\n{R_arm_from_sensor_rounded}")
    print(f"euler_arm_from_sensor: {euler_arm_from_sensor}")
    print(f"t_arm_from_sensor: {t_arm_from_sensor}")

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
                            t_arm_from_robot = np.array([x, y, z])
                            R_vec_arm_from_robot = np.array([rx, ry, rz])
                            R_arm_from_robot = R.from_rotvec(R_vec_arm_from_robot, degrees=True).as_matrix()

                            # 同次変換行列を作成
                            T_arm_from_robot_transform = Transform(R_arm_from_robot, t_arm_from_robot)

                            # --- 姿勢の計算 ---
                            # ロボット座標系におけるセンサー姿勢をアーム姿勢に変換
                            # 計算式：T_arm_from_robot = T_sensor_from_robot @ T_arm_from_sensor  =>  T_sensor_from_robot = T_arm_from_robot @ T_arm_from_sensor.inv()
                            T_sensor_from_robot_transform = T_arm_from_robot_transform @ T_arm_from_sensor_transform.inv()
                            # 計算式：T_sensor_from_robot = T_aurora_from_robot @ T_sensor_from_aurora =>  T_sensor_from_aurora = T_aurora_from_robot.inv() @ T_sensor_from_robot
                            T_sensor_from_aurora_transform = T_aurora_from_robot_transform.inv() @ T_sensor_from_robot_transform

                            t_sensor_from_aurora = T_sensor_from_aurora_transform.t
                            R_sensor_from_aurora = T_sensor_from_aurora_transform.R
                            quaternion_sensor_from_aurora = R.from_matrix(R_sensor_from_aurora).as_quat()

                            # 位置座標にノイズの追加
                            noisy_t_sensor_from_aurora = t_sensor_from_aurora.copy()
                            if add_noise > 0:
                                noisy_t_sensor_from_aurora += np.random.normal(0, add_noise, 3)
                            
                            # クォータニオンにノイズの追加
                            noisy_quaternion_sensor_from_aurora = quaternion_sensor_from_aurora.copy()
                            if add_quaternion_noise > 0:
                                quaternion_noise = np.random.normal(0, add_quaternion_noise, 4)
                                noisy_quaternion_sensor_from_aurora += quaternion_noise
                                quaternion_norm = np.linalg.norm(noisy_quaternion_sensor_from_aurora)
                                if quaternion_norm > 0:
                                    noisy_quaternion_sensor_from_aurora /= quaternion_norm

                            # データポイントを追加
                            data_points.append([
                                round(x, 5), round(y, 5), round(z, 5),
                                round(rx, 5), round(ry, 5), round(rz, 5),
                                round(noisy_t_sensor_from_aurora[0], 5), round(noisy_t_sensor_from_aurora[1], 5), round(noisy_t_sensor_from_aurora[2], 5),
                                round(noisy_quaternion_sensor_from_aurora[0], 5), round(noisy_quaternion_sensor_from_aurora[1], 5), round(noisy_quaternion_sensor_from_aurora[2], 5), round(noisy_quaternion_sensor_from_aurora[3], 5),
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

def generate_output_filename(euler_aurora_from_robot, t_aurora_from_robot, 
                             euler_arm_from_sensor, t_arm_from_sensor, 
                             add_noise, add_quaternion_noise=0):
    """
    パラメータから動的にファイル名を生成する関数
    """
    rot_str = "-".join(map(str, euler_aurora_from_robot))
    trans_str = "-".join(map(str, t_aurora_from_robot))
    sen_rot_str = "-".join(map(str, euler_arm_from_sensor))
    sen_trans_str = "-".join(map(str, t_arm_from_sensor))

    filename = (
        f"robot&aurora/current_code/new_transform/data/"
        f"AUfRO_R{rot_str}_T{trans_str}_"
        f"ARfSE-R{sen_rot_str}_T{sen_trans_str}_"
        f"n{add_noise}_qn{add_quaternion_noise}.csv"
    )
    
    return filename

def main(
    output_file,
    x_range,
    y_range,
    z_range,
    rx_range,
    ry_range,
    rz_range,
    euler_aurora_from_robot=None,
    t_aurora_from_robot=None,
    euler_arm_from_sensor=None,
    t_arm_from_sensor=None,
    step_size=None,
    rot_step_size=None,
    add_noise=0,
    add_quaternion_noise=0
):
    """
    メイン処理を実行する関数
    """
    # デフォルトパラメータの設定
    if euler_aurora_from_robot is None:euler_aurora_from_robot = [0, 0, 0]
    if t_aurora_from_robot is None:t_aurora_from_robot = [0, 0, 0]
    if euler_arm_from_sensor is None:euler_arm_from_sensor = [0, 0, 0]
    if t_arm_from_sensor is None:t_arm_from_sensor = [0, 0, 0]
    if step_size is None:step_size = 20
    if rot_step_size is None:rot_step_size = 30
    if add_noise is None:add_noise = 0
    if add_quaternion_noise is None:add_quaternion_noise = 0

    return generate_synthetic_data(
        output_file=output_file,
        euler_aurora_from_robot=euler_aurora_from_robot,
        t_aurora_from_robot=t_aurora_from_robot,
        euler_arm_from_sensor=euler_arm_from_sensor,
        t_arm_from_sensor=t_arm_from_sensor,
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        rx_range=rx_range,
        ry_range=ry_range,
        rz_range=rz_range,
        step_size=step_size,
        rot_step_size=rot_step_size,
        add_noise=add_noise,
        add_quaternion_noise=add_quaternion_noise
    )

if __name__ == "__main__":

    # ====パラメータを設定====
    # 範囲
    x_range = (75, 175)      # X座標の範囲（mm）
    y_range = (-50, 50)      # Y座標の範囲（mm）
    z_range = (-350, -250)   # Z座標の範囲（mm）
    rx_range = (0, 0)         # ロボットアームのrx範囲（度）
    ry_range = (0, 0)         # ロボットアームのry範囲（度）
    rz_range = (0, 0)         # ロボットアームのrz範囲（度）
    # Auroraとロボットの関係性
    euler_aurora_from_robot = [0, 0, 0]  # ロボット座標系から見たAurora座標系の姿勢を表すオイラー角（度）[roll, pitch, yaw]
    t_aurora_from_robot = [0, 0, 0]    # ロボット座標系から見たAurora座標系への並進ベクトル（mm）[tx, ty, tz]
    # センサーとアームの関係性
    euler_arm_from_sensor = [0, 0, 0] # センサー座標系から見たアーム座標系の姿勢を表すオイラー角（度）[roll, pitch, yaw]
    t_arm_from_sensor = [10, 20, 30] # センサー座標系からアーム座標系への並進ベクトル（mm）[tx, ty, tz]
    # ステップサイズ
    step_size = 20              # 位置のステップサイズ（mm）
    rot_step_size = 30          # 回転のステップサイズ（度）
    # ノイズ
    add_noise = 0              # 位置ノイズの標準偏差（mm）
    add_quaternion_noise = 0   # クォータニオンノイズの標準偏差
    # ===========================
    
    # 動的にファイル名を生成
    output_file = generate_output_filename(
        euler_aurora_from_robot,
        t_aurora_from_robot,
        euler_arm_from_sensor,
        t_arm_from_sensor,
        add_noise,
        add_quaternion_noise
    )
    
    print(f"Generated filename: {output_file}")
    
    # main関数の呼び出し
    result = main(
        output_file=output_file,
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        rx_range=rx_range,
        ry_range=ry_range,
        rz_range=rz_range,
        euler_aurora_from_robot=euler_aurora_from_robot,
        t_aurora_from_robot=t_aurora_from_robot,
        euler_arm_from_sensor=euler_arm_from_sensor,
        t_arm_from_sensor=t_arm_from_sensor,
        step_size=step_size,
        rot_step_size=rot_step_size,
        add_noise=add_noise,
        add_quaternion_noise=add_quaternion_noise
    )