import numpy as np
from scipy.spatial.transform import Rotation as R
import csv
import os

def generate_synthetic_data(
    rotation_euler_deg, 
    translation_vector, 
    robot_arm_euler_deg,
    arm_sensor_euler_deg,
    x_range, 
    y_range, 
    z_range, 
    num_points_per_axis=None, 
    step_size=None, 
    output_file="robot&aurora/current_code/calibration_data/aurora_robot_pose_log.csv",
    add_noise=0
):
    """
    ロボット座標系とAurora座標系の変換関係に基づいて合成データを生成する
    
    :param rotation_euler_deg: 回転角度（オイラー角、度数法）[rx, ry, rz]（固定軸回転XYZ）
    :param translation_vector: 並進ベクトル [tx, ty, tz]
    :param x_range: X座標の範囲 (開始値, 終了値)
    :param y_range: Y座標の範囲 (開始値, 終了値)
    :param z_range: Z座標の範囲 (開始値, 終了値)
    :param num_points_per_axis: 各軸の点の数
    :param step_size: サンプリング間隔（mm）
    :param output_file: 出力CSVファイル名
    :param add_noise: Aurora座標に追加するガウスノイズの標準偏差（mm）
    :return: 生成したデータポイント
    """
    # 回転と並進を変換行列に変換（固定軸回転XYZ）
    rotation = R.from_euler('XYZ', rotation_euler_deg, degrees=True)
    rotation_matrix = rotation.as_matrix()
    
    # サンプリング点を生成
    if num_points_per_axis is not None:
        # 点の数が指定された場合
        x_points = np.linspace(x_range[0], x_range[1], num_points_per_axis)
        y_points = np.linspace(y_range[0], y_range[1], num_points_per_axis)
        z_points = np.linspace(z_range[0], z_range[1], num_points_per_axis)
    elif step_size is not None:
        # 間隔が指定された場合
        x_points = np.arange(x_range[0], x_range[1] + step_size/2, step_size)  # 浮動小数点誤差を考慮
        y_points = np.arange(y_range[0], y_range[1] + step_size/2, step_size)
        z_points = np.arange(z_range[0], z_range[1] + step_size/2, step_size)
    else:
        raise ValueError("Either num_points_per_axis or step_size must be specified")
    
    # 出力するAurora姿勢情報の計算
    arm_sensor_rotation = R.from_euler('XYZ', arm_sensor_euler_deg, degrees=True)
    arm_sensor_rotation_matrix = arm_sensor_rotation.as_matrix()
    robot_arm_rotation = R.from_euler('XYZ', robot_arm_euler_deg, degrees=True)
    robot_arm_rotation_matrix = robot_arm_rotation.as_matrix()
    robot_sensor_pose = robot_arm_rotation_matrix @ arm_sensor_rotation_matrix
    robot_sensor_rotation = R.from_matrix(robot_sensor_pose)
    robot_sensor_quaternion = robot_sensor_rotation.as_quat()
    robot_sensor_euler_deg = robot_sensor_rotation.as_euler('XYZ', degrees=True)
    print(f"robot_sensor_euler_deg: {robot_sensor_euler_deg}")

    # 出力ディレクトリの確保
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # データを格納するリスト
    data_points = []
    
    # 全ての点の組み合わせに対してAurora座標を計算
    for x in x_points:
        for y in y_points:
            for z in z_points:
                # ロボット座標
                robot_point = np.array([x, y, z])
                
                # ロボット座標からAurora座標への変換
                # Aurora = R * Robot + T
                aurora_point = rotation_matrix @ robot_point + translation_vector
                
                # ノイズの追加（指定がある場合）
                if add_noise > 0:
                    noise = np.random.normal(0, add_noise, 3)
                    aurora_point += noise
                
                # データポイントを追加
                data_points.append([
                    x, y, z,
                    robot_arm_euler_deg[0], robot_arm_euler_deg[1], robot_arm_euler_deg[2],
                    aurora_point[0], aurora_point[1], aurora_point[2],
                    robot_sensor_quaternion[0], robot_sensor_quaternion[1], robot_sensor_quaternion[2], robot_sensor_quaternion[3]
                ])
    
    # CSVファイルに書き込み
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['robot_x', 'robot_y', 'robot_z', 'robot_roll', 'robot_pitch', 'robot_yaw', 'aurora_x', 'aurora_y', 'aurora_z', 'aurora_quat_x', 'aurora_quat_y', 'aurora_quat_z', 'aurora_quat_w'])
        writer.writerows(data_points)
    
    print(f"生成したデータポイント数: {len(data_points)}")
    print(f"データを {output_file} に保存しました")
    
    return data_points

def main(
    x_range,
    y_range,
    z_range,
    rotation_euler_deg=None,
    translation_vector=None, 
    robot_arm_euler_deg=None,
    arm_sensor_euler_deg=None,
    num_points_per_axis=None,
    step_size=None,
    output_file="robot&aurora/current_code/calibration_data/aurora_robot_pose_log.csv",
    add_noise=0
):
    """
    メイン処理を実行する関数
    
    :param x_range: X座標の範囲 (開始値, 終了値)
    :param y_range: Y座標の範囲 (開始値, 終了値)
    :param z_range: Z座標の範囲 (開始値, 終了値)
    :param rotation_euler_deg: 回転角度（オイラー角、度数法）[rx, ry, rz]
    :param translation_vector: 並進ベクトル [tx, ty, tz]
    :param num_points_per_axis: 各軸の点の数
    :param step_size: サンプリング間隔（mm）
    :param output_file: 出力CSVファイル名
    :param add_noise: Aurora座標に追加するガウスノイズの標準偏差（mm）
    :return: 生成したデータポイント
    """
    # デフォルトパラメータの設定
    if rotation_euler_deg is None:
        rotation_euler_deg = [0, 0, 90]  # デフォルトのオイラー角（度数法）
    
    if translation_vector is None:
        translation_vector = [0, 0, -100]  # デフォルトの並進ベクトル
    
    # 点数または間隔のチェック
    if num_points_per_axis is None and step_size is None:
        step_size = 20  # デフォルトの間隔（mm）
    
    # データの生成
    return generate_synthetic_data(
        rotation_euler_deg=rotation_euler_deg,
        translation_vector=translation_vector,
        robot_arm_euler_deg=robot_arm_euler_deg,
        arm_sensor_euler_deg=arm_sensor_euler_deg,
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        num_points_per_axis=num_points_per_axis,
        step_size=step_size,
        output_file=output_file,
        add_noise=add_noise
    )

if __name__ == "__main__":
    # ここで全てのパラメータを一か所で設定（ここだけを変更すれば良い）
    result = main(
        x_range=(250, 350),                    # X座標の範囲 (開始値, 終了値)
        y_range=(-50, 50),                     # Y座標の範囲 (開始値, 終了値)
        z_range=(75, 175),                     # Z座標の範囲 (開始値, 終了値)
        rotation_euler_deg=[56, -31, -88],         # 回転角度（オイラー角XYZ、度数法）＿ロボット座標系をどれだけ回転したらAurora座標系になるか
        translation_vector=[-20, 44, 99],      # 並進ベクトル [tx, ty, tz]＿ロボット座標系原点をどれだけ平行移動したらAurora座標系原点に重なるか
        robot_arm_euler_deg=[180, 0, 0],         # ロボットアームのオイラー角（度数法）
        arm_sensor_euler_deg=[-90, 0, 0],         # アームとセンサのオイラー角（度数法）＿アーム座標系をどれだけ回転したらセンサ座標系になるか
        step_size=10,                          # サンプリング間隔（mm）
        # num_points_per_axis=5,               # 各軸の点の数（指定する場合はstep_sizeをコメントアウト）
        output_file="robot&aurora/current_code/calibration_data/aurora_robot_pose_log.csv",  # 出力CSVファイル名
        add_noise=0                          # Aurora座標に追加するノイズの標準偏差（mm）
    )