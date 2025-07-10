import numpy as np
from scipy.spatial.transform import Rotation as R
import csv
import os

def generate_synthetic_data(
    rotation_euler_deg_aur2rob, 
    translation_vector_aur2rob, 
    robot_arm_euler_deg,
    rotation_euler_deg_sen2arm,
    x_range, 
    y_range, 
    z_range, 
    step_size=None, 
    output_file="robot&aurora/current_code/calibration_data/aurora_robot_pose_log.csv",
    add_noise=0,
    add_quaternion_noise=0
):
    """
    ロボット座標系とAurora座標系の変換関係に基づいて合成データを生成する
    
    :param rotation_euler_deg_aur2rob: 回転角度（オイラー角、度数法）[rx, ry, rz]（固定軸回転zyx）
    :param translation_vector_aur2rob: 並進ベクトル [tx, ty, tz]
    :param x_range: X座標の範囲 (開始値, 終了値)
    :param y_range: Y座標の範囲 (開始値, 終了値)
    :param z_range: Z座標の範囲 (開始値, 終了値)
    :param step_size: サンプリング間隔（mm）
    :param output_file: 出力CSVファイル名
    :param add_noise: Aurora座標に追加するガウスノイズの標準偏差（mm）
    :param add_quaternion_noise: クォータニオンに追加するガウスノイズの標準偏差
    :return: 生成したデータポイント
    """

    euler_aur2rob_ypr = [rotation_euler_deg_aur2rob[2],
                         rotation_euler_deg_aur2rob[1],
                         rotation_euler_deg_aur2rob[0]]  # Aurora→Robotのオイラー角をYPR順に変換 
    # 回転と並進を変換行列に変換（固定軸回転zyx）
    rotation_aur2rob = R.from_euler('zyx', euler_aur2rob_ypr, degrees=True)
    R_matrix_aur2rob = rotation_aur2rob.as_matrix()
    # 回転行列を小数点第5位で四捨五入して表示
    R_matrix_aur2rob_rounded = np.round(R_matrix_aur2rob, 5)
    print(f"R_matrix_aur2rob:\n{R_matrix_aur2rob_rounded}")
    print(f"rotation_euler_deg_aur2rob: {rotation_euler_deg_aur2rob}")
    print(f"rotation_euler_deg_sen2arm: {rotation_euler_deg_sen2arm}")
    
    # サンプリング点を生成
    if step_size is not None:
        # 間隔が指定された場合
        x_points = np.arange(x_range[0], x_range[1] + step_size/2, step_size)  # 浮動小数点誤差を考慮
        y_points = np.arange(y_range[0], y_range[1] + step_size/2, step_size)
        z_points = np.arange(z_range[0], z_range[1] + step_size/2, step_size)
    else:
        raise ValueError("Either step_size must be specified")
    
    euler_sen2arm_ypr = [rotation_euler_deg_sen2arm[2],
                         rotation_euler_deg_sen2arm[1],
                         rotation_euler_deg_sen2arm[0]]  # センサー→アームのオイラー角をYPR順に変換
    # 出力するAurora姿勢情報の計算
    # センサー座標系からアーム座標系への回転
    rotation_sen2arm = R.from_euler('zyx', euler_sen2arm_ypr, degrees=True)
    R_matrix_sen2arm = rotation_sen2arm.as_matrix()
    # センサー→アーム回転行列を小数点第5位で四捨五入して表示
    R_matrix_sen2arm_rounded = np.round(R_matrix_sen2arm, 5)
    print(f"R_matrix_sen2arm:\n{R_matrix_sen2arm_rounded}")
    
    robot_arm_euler_ypr = [robot_arm_euler_deg[2],
                           robot_arm_euler_deg[1],
                           robot_arm_euler_deg[0]]  # ロボットアームのオイラー角をYPR順に変換
    # ロボットアームの回転
    rotation_arm = R.from_euler('zyx', robot_arm_euler_ypr, degrees=True)
    R_matrix_arm = rotation_arm.as_matrix()
    # ロボット座標系におけるセンサー姿勢をアーム姿勢に変換
    R_matrix_sensor_from_robot = R_matrix_sen2arm.T @ R_matrix_arm
    R_matrix_sensor_from_aurora = R_matrix_aur2rob.T @ R_matrix_sensor_from_robot
    rotation_sensor_from_aurora = R.from_matrix(R_matrix_sensor_from_aurora)
    # センサー姿勢をオイラー角とクォータニオンに変換
    quaternion_sensor_from_aurora = rotation_sensor_from_aurora.as_quat()
    euler_sensor_from_aurora_ypr = rotation_sensor_from_aurora.as_euler('zyx', degrees=True)
    euler_sensor_from_aurora = [euler_sensor_from_aurora_ypr[2],
                                 euler_sensor_from_aurora_ypr[1],
                                 euler_sensor_from_aurora_ypr[0]]  # RPY順に変換
    print(f"euler_sensor_from_aurora: {euler_sensor_from_aurora}")

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
                # Aurora = R_aur2rob.inv * (Robot - T_aur2rob) = R_aur2rob.T @ (Robot - T_aur2rob)
                aurora_point = R_matrix_aur2rob.T @ (robot_point - translation_vector_aur2rob)

                # 位置座標にノイズの追加（指定がある場合）
                if add_noise > 0:
                    position_noise = np.random.normal(0, add_noise, 3)
                    aurora_point += position_noise
                
                # クォータニオンのコピーを作成
                noisy_quaternion = quaternion_sensor_from_aurora.copy()
                
                # クォータニオンにノイズの追加（指定がある場合）
                if add_quaternion_noise > 0:
                    # クォータニオンにガウシアンノイズを追加
                    quaternion_noise = np.random.normal(0, add_quaternion_noise, 4)
                    noisy_quaternion += quaternion_noise
                    
                    # クォータニオンを正規化（単位クォータニオンにする）
                    quaternion_norm = np.linalg.norm(noisy_quaternion)
                    if quaternion_norm > 0:  # ゼロ除算を避ける
                        noisy_quaternion = noisy_quaternion / quaternion_norm
                    else:
                        # 万が一ノルムが0になった場合は、元のクォータニオンを使用
                        noisy_quaternion = quaternion_sensor_from_aurora.copy()
                
                # データポイントを追加（小数点第5位で四捨五入）
                data_points.append([
                    round(x, 5), round(y, 5), round(z, 5),
                    round(robot_arm_euler_deg[0], 5), round(robot_arm_euler_deg[1], 5), round(robot_arm_euler_deg[2], 5),
                    round(aurora_point[0], 5), round(aurora_point[1], 5), round(aurora_point[2], 5),
                    round(noisy_quaternion[0], 5), round(noisy_quaternion[1], 5), round(noisy_quaternion[2], 5), round(noisy_quaternion[3], 5)
                ])
    
    # CSVファイルに書き込み
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['robot_x', 'robot_y', 'robot_z', 'robot_roll', 'robot_pitch', 'robot_yaw', 'aurora_x', 'aurora_y', 'aurora_z', 'aurora_quat_x', 'aurora_quat_y', 'aurora_quat_z', 'aurora_quat_w'])
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
    rotation_euler_deg_aur2rob=None,
    translation_vector_aur2rob=None, 
    robot_arm_euler_deg=None,
    rotation_euler_deg_sen2arm=None,
    step_size=None,
    output_file="robot&aurora/current_code/calibration_data/aurora_robot_pose_log.csv",
    add_noise=0,
    add_quaternion_noise=0
):
    """
    メイン処理を実行する関数
    
    :param x_range: X座標の範囲 (開始値, 終了値)
    :param y_range: Y座標の範囲 (開始値, 終了値)
    :param z_range: Z座標の範囲 (開始値, 終了値)
    :param rotation_euler_deg_aur2rob: 回転角度（オイラー角、度数法）[rx, ry, rz]
    :param translation_vector_aur2rob: 並進ベクトル [tx, ty, tz]
    :param step_size: サンプリング間隔（mm）
    :param output_file: 出力CSVファイル名
    :param add_noise: Aurora座標に追加するガウスノイズの標準偏差（mm）
    :param add_quaternion_noise: クォータニオンに追加するガウスノイズの標準偏差
    :return: 生成したデータポイント
    """
    # デフォルトパラメータの設定
    if rotation_euler_deg_aur2rob is None:
        rotation_euler_deg_aur2rob = [0, 0, 90]  # デフォルトのオイラー角（度数法）
    
    if translation_vector_aur2rob is None:
        translation_vector_aur2rob = [0, 0, -100]  # デフォルトの並進ベクトル
    
    # 点数または間隔のチェック
    if step_size is None:
        step_size = 20  # デフォルトの間隔（mm）
    
    # データの生成
    return generate_synthetic_data(
        rotation_euler_deg_aur2rob=rotation_euler_deg_aur2rob,
        translation_vector_aur2rob=translation_vector_aur2rob,
        robot_arm_euler_deg=robot_arm_euler_deg,
        rotation_euler_deg_sen2arm=rotation_euler_deg_sen2arm,
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        step_size=step_size,
        output_file=output_file,
        add_noise=add_noise,
        add_quaternion_noise=add_quaternion_noise
    )


def generate_output_filename(rotation_euler_deg_aur2rob, translation_vector_aur2rob, 
                            robot_arm_euler_deg, rotation_euler_deg_sen2arm, add_noise, add_quaternion_noise=0):
    """
    パラメータから動的にファイル名を生成する関数
    
    Args:
        rotation_euler_deg_aur2rob: Aurora→Robot回転角度 [rx, ry, rz]
        translation_vector_aur2rob: Aurora→Robot並進ベクトル [tx, ty, tz]
        robot_arm_euler_deg: ロボットアームのオイラー角 [rx, ry, rz]
        rotation_euler_deg_sen2arm: Sensor→Arm回転角度 [rx, ry, rz]
        add_noise: 位置ノイズの標準偏差
        add_quaternion_noise: クォータニオンノイズの標準偏差
    
    Returns:
        str: 生成されたファイル名
    """
    # 各パラメータを文字列に変換（負の値も含めて）
    rot_str = "-".join(map(str, rotation_euler_deg_aur2rob))
    trans_str = "-".join(map(str, translation_vector_aur2rob))
    arm_str = "-".join(map(str, robot_arm_euler_deg))
    sen_str = "-".join(map(str, rotation_euler_deg_sen2arm))
    
    # ファイル名を生成（クォータニオンノイズのパラメータも追加）
    filename = f"robot&aurora/current_code/calibration_data/pose_R{rot_str}_T{trans_str}_ARM{arm_str}_SEN{sen_str}_n{add_noise}_qn{add_quaternion_noise}.csv"
    
    return filename

if __name__ == "__main__":
    # パラメータを一か所で設定
    rotation_euler_deg_aur2rob = [30,   120,  -45]  # Aurora座標系からロボット座標系への回転角度（度）[rx, ry, rz]
    translation_vector_aur2rob = [0,0,0]  # Aurora座標系からロボット座標系への並進ベクトル（mm）[tx, ty, tz]
    robot_arm_euler_deg = [50, 120, 180]  # ロボットアームのオイラー角（度）[roll, pitch, yaw]
    rotation_euler_deg_sen2arm = [  -60,   -80,  150]  # センサー座標系からアーム座標系への回転角度（度）[rx, ry, rz]
    add_noise = 0  # 位置ノイズの標準偏差（mm）
    add_quaternion_noise = 0  # クォータニオンノイズの標準偏差
    
    # 動的にファイル名を生成
    output_file = generate_output_filename(
        rotation_euler_deg_aur2rob,
        translation_vector_aur2rob,
        robot_arm_euler_deg,
        rotation_euler_deg_sen2arm,
        add_noise,
        add_quaternion_noise
    )
    
    print(f"Generated filename: {output_file}")
    
    # main関数の呼び出し
    result = main(
        x_range=(75, 175),  # X座標の範囲（mm）[開始値, 終了値]
        y_range=(-50, 50),   # Y座標の範囲（mm）[開始値, 終了値]
        z_range=(-350, -250),   # Z座標の範囲（mm）[開始値, 終了値]
        rotation_euler_deg_aur2rob=rotation_euler_deg_aur2rob,
        translation_vector_aur2rob=translation_vector_aur2rob,
        robot_arm_euler_deg=robot_arm_euler_deg,
        rotation_euler_deg_sen2arm=rotation_euler_deg_sen2arm,
        step_size=20,
        output_file=output_file,  # 動的に生成されたファイル名を使用
        add_noise=add_noise,
        add_quaternion_noise=add_quaternion_noise
    )