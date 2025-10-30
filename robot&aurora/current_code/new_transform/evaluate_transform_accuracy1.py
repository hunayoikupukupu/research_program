# 座標・姿勢変換の正確性を評価するためのデータ収集スクリプト
import time
import numpy as np
import csv
from utils.pose_formatter import generateRobotArmAxisAngle, generateProbe
from utils.initialization import initialize_robot, initialize_aurora
from scipy.spatial.transform import Rotation as R
from calibration.transformation_utils import Transform, compute_transform_difference
from main import main as run_pose_transoformation


def collect_diff_data(arm, aurora, x_range, y_range, z_range, N):
    """指定された範囲でデータを収集"""
    x_start, x_end = x_range
    y_start, y_end = y_range
    z_start, z_end = z_range
    
    x_step = (x_end - x_start) / N
    y_step = (y_end - y_start) / N
    z_step = (z_end - z_start) / N
    
    total_points = (N+1)**3
    
    # データ格納用ディクショナリ
    robot_data = {
        'x': [], 'y': [], 'z': [], 
        'rx': [], 'ry': [], 'rz': []
    }
    
    aurora_data = {
        'x': [], 'y': [], 'z': [], 
        'quat_x': [], 'quat_y': [], 'quat_z': [], 'quat_w': [],
        'quality': []
    }

    robot_after_data = {
        'x': [], 'y': [], 'z': [],
        'rx': [], 'ry': [], 'rz': []
    }

    aurora_after_data = {
        'x': [], 'y': [], 'z': [],
        'quat_x': [], 'quat_y': [], 'quat_z': [], 'quat_w': [],
        'quality': []
    }
    
    print(f"データ収集開始: 合計 {total_points} ポイント")
    
    # 各位置での計測
    for point in range(total_points):
        # インデックスから3次元座標を計算
        i = point // ((N+1)**2)
        j = (point // (N+1)) % (N+1)
        k = point % (N+1)
        
        # 現在のポイント位置を計算
        current_x = x_start + x_step * i
        current_y = y_start + y_step * j
        current_z = z_start + z_step * k
        
        # 進捗表示
        if point % 10 == 0:
            progress = (point / total_points) * 100
            print(f"進捗: {progress:.1f}% ({point}/{total_points})")
        
        # ロボットアームを移動
        arm.set_position(x=current_x, y=current_y, z=current_z, roll=0, pitch=0, yaw=180, speed=50, wait=True)
        time.sleep(2)
        
        # データ取得
        robot = generateRobotArmAxisAngle(arm.get_position_aa())
        probes = generateProbe(aurora.get_frame())
        
        # ロボットデータを保存
        robot_data['x'].append(robot.pos.x)
        robot_data['y'].append(robot.pos.y)
        robot_data['z'].append(robot.pos.z)
        robot_data['rx'].append(robot.rot.rx)
        robot_data['ry'].append(robot.rot.ry)
        robot_data['rz'].append(robot.rot.rz)

        # オーロラデータを保存
        aurora_data['x'].append(probes[0].pos.x)
        aurora_data['y'].append(probes[0].pos.y)
        aurora_data['z'].append(probes[0].pos.z)
        aurora_data['quat_x'].append(probes[0].quat.x)
        aurora_data['quat_y'].append(probes[0].quat.y)
        aurora_data['quat_z'].append(probes[0].quat.z)
        aurora_data['quat_w'].append(probes[0].quat.w)
        aurora_data['quality'].append(probes[0].quality)

        time.sleep(1)

        # 現在のauroraの位置・姿勢を目標にセットし、それを実現するロボットアームの姿勢を計算
        t_sensor_from_aurora_goal = np.array([probes[0].pos.x, probes[0].pos.y, probes[0].pos.z])
        quat_sensor_from_aurora_goal = np.array([probes[0].quat.x, probes[0].quat.y, probes[0].quat.z, probes[0].quat.w])

        T_arm_from_robot = run_pose_transoformation(
            goal_aurora_point=t_sensor_from_aurora_goal,
            goal_aurora_quaternion=quat_sensor_from_aurora_goal,
            world_calib_csv="robot&aurora/current_code/new_transform/data/aurora_robot_pose_log_202510281545.csv",
            hand_eye_calib_csv="robot&aurora/current_code/new_transform/data/aurora_robot_orientation_log_2020510281902.csv"
        )

        # T_arm_from_robotをTransformオブジェクトに変換
        T_arm_from_robot_transform = Transform.from_matrix(T_arm_from_robot)
        t_arm_from_robot = T_arm_from_robot_transform.t
        R_arm_from_robot = T_arm_from_robot_transform.R
        arm_rotvec_from_robot = R.from_matrix(R_arm_from_robot).as_rotvec(degrees=True)

        # ロボットアームを移動
        angle_pose = [t_arm_from_robot[0], t_arm_from_robot[1], t_arm_from_robot[2], arm_rotvec_from_robot[0], arm_rotvec_from_robot[1], arm_rotvec_from_robot[2]]
        arm.set_position_aa(angle_pose, speed=50, wait=True)

        robot_after = generateRobotArmAxisAngle(arm.get_position_aa())
        probes_after = generateProbe(aurora.get_frame())

        # 移動後のロボットデータを保存
        robot_after_data['x'].append(robot_after.pos.x)
        robot_after_data['y'].append(robot_after.pos.y)
        robot_after_data['z'].append(robot_after.pos.z)
        robot_after_data['rx'].append(robot_after.rot.rx)
        robot_after_data['ry'].append(robot_after.rot.ry)
        robot_after_data['rz'].append(robot_after.rot.rz)

        # 移動後のオーロラデータを保存
        aurora_after_data['x'].append(probes_after[0].pos.x)
        aurora_after_data['y'].append(probes_after[0].pos.y)
        aurora_after_data['z'].append(probes_after[0].pos.z)
        aurora_after_data['quat_x'].append(probes_after[0].quat.x)
        aurora_after_data['quat_y'].append(probes_after[0].quat.y)
        aurora_after_data['quat_z'].append(probes_after[0].quat.z)
        aurora_after_data['quat_w'].append(probes_after[0].quat.w)
        aurora_after_data['quality'].append(probes_after[0].quality)

        time.sleep(2)
    
    print("データ収集完了")
    return robot_data, aurora_data, robot_after_data, aurora_after_data

# 正確性評価用の関数
def evaluate_accuracy(robot_data, aurora_data, robot_after_data, aurora_after_data):
    """データの正確性を評価"""

    # 誤差データ格納用ディクショナリ
    delta_t_aurora_data = {
        'x': [], 'y': [], 'z': [],
        'norm': []
    }
    delta_R_aurora_data = {
        'rx': [], 'ry': [], 'rz': [],
        'angle': []
    }
    delta_t_robot_data = {
        'x': [], 'y': [], 'z': [],
        'norm': []
    }
    delta_R_robot_data = {
        'rx': [], 'ry': [], 'rz': [],
        'angle': []
    }

    total_points = len(robot_data['x'])
    for i in range(total_points):

        # auroraの差分計算
        # 目標のT_sensor_from_auroraを作成
        t_sensor_from_aurora_goal = np.array([aurora_data['x'][i], aurora_data['y'][i], aurora_data['z'][i]])
        quat_sensor_from_aurora_goal = np.array([aurora_data['quat_x'][i], aurora_data['quat_y'][i], aurora_data['quat_z'][i], aurora_data['quat_w'][i]])
        R_sensor_from_aurora_goal = R.from_quat(quat_sensor_from_aurora_goal).as_matrix()
        T_sensor_from_aurora_goal = Transform(R_sensor_from_aurora_goal, t_sensor_from_aurora_goal).matrix

        # 移動後のT_sensor_from_auroraを作成
        t_sensor_from_aurora_after = np.array([aurora_after_data['x'][i], aurora_after_data['y'][i], aurora_after_data['z'][i]])
        quat_sensor_from_aurora_after = np.array([aurora_after_data['quat_x'][i], aurora_after_data['quat_y'][i], aurora_after_data['quat_z'][i], aurora_after_data['quat_w'][i]])
        R_sensor_from_aurora_after = R.from_quat(quat_sensor_from_aurora_after).as_matrix()
        T_sensor_from_aurora_after = Transform(R_sensor_from_aurora_after, t_sensor_from_aurora_after).matrix

        # 差分を計算
        delta_t_aurora, delta_rotvec_aurora, delta_t_norm_aurora, delta_angle_aurora = compute_transform_difference(T_sensor_from_aurora_goal, T_sensor_from_aurora_after)

        # robotの差分計算
        # 目標のT_arm_from_robotを作成
        t_arm_from_robot_goal = np.array([robot_data['x'][i], robot_data['y'][i], robot_data['z'][i]])
        rotvec_arm_from_robot_goal = np.array([robot_data['rx'][i], robot_data['ry'][i], robot_data['rz'][i]])
        R_arm_from_robot_goal = R.from_rotvec(rotvec_arm_from_robot_goal, degrees=True).as_matrix()
        T_arm_from_robot_goal = Transform(R_arm_from_robot_goal, t_arm_from_robot_goal).matrix

        # 移動後のT_arm_from_robotを作成
        t_arm_from_robot_after = np.array([robot_after_data['x'][i], robot_after_data['y'][i], robot_after_data['z'][i]])
        rotvec_arm_from_robot_after = np.array([robot_after_data['rx'][i], robot_after_data['ry'][i], robot_after_data['rz'][i]])
        R_arm_from_robot_after = R.from_rotvec(rotvec_arm_from_robot_after, degrees=True).as_matrix()
        T_arm_from_robot_after = Transform(R_arm_from_robot_after, t_arm_from_robot_after).matrix

        # 差分を計算
        delta_t_robot, delta_rotvec_robot, delta_t_norm_robot, delta_angle_robot = compute_transform_difference(T_arm_from_robot_goal, T_arm_from_robot_after)

        # auroraの差分データを保存
        delta_t_aurora_data['x'].append(delta_t_aurora[0])
        delta_t_aurora_data['y'].append(delta_t_aurora[1])
        delta_t_aurora_data['z'].append(delta_t_aurora[2])
        delta_t_aurora_data['norm'].append(delta_t_norm_aurora)

        delta_R_aurora_data['rx'].append(delta_rotvec_aurora[0])
        delta_R_aurora_data['ry'].append(delta_rotvec_aurora[1])
        delta_R_aurora_data['rz'].append(delta_rotvec_aurora[2])
        delta_R_aurora_data['angle'].append(delta_angle_aurora)

        # robotの差分データを保存
        delta_t_robot_data['x'].append(delta_t_robot[0])
        delta_t_robot_data['y'].append(delta_t_robot[1])
        delta_t_robot_data['z'].append(delta_t_robot[2])
        delta_t_robot_data['norm'].append(delta_t_norm_robot)

        delta_R_robot_data['rx'].append(delta_rotvec_robot[0])
        delta_R_robot_data['ry'].append(delta_rotvec_robot[1])
        delta_R_robot_data['rz'].append(delta_rotvec_robot[2])
        delta_R_robot_data['angle'].append(delta_angle_robot)

    return delta_t_aurora_data, delta_R_aurora_data, delta_t_robot_data, delta_R_robot_data

def save_to_csv_extended(robot_data, aurora_data, robot_after_data, aurora_after_data, delta_t_aurora_data, delta_R_aurora_data, delta_t_robot_data, delta_R_robot_data, filename, decimal_places=3):
    """拡張データをCSVファイルに保存"""
    # 出力用の2次元リスト作成
    data = []
    total_points = len(robot_data['x'])
    
    for i in range(total_points):
        row = [
            # ロボットの位置データ
            np.round(robot_data['x'][i], decimal_places),
            np.round(robot_data['y'][i], decimal_places),
            np.round(robot_data['z'][i], decimal_places),
            
            # ロボットのオイラー角データ
            np.round(robot_data['rx'][i], decimal_places),
            np.round(robot_data['ry'][i], decimal_places),
            np.round(robot_data['rz'][i], decimal_places),

            # オーロラの位置データ
            np.round(aurora_data['x'][i], decimal_places),
            np.round(aurora_data['y'][i], decimal_places),
            np.round(aurora_data['z'][i], decimal_places),
            
            # オーロラのクォータニオンデータ
            np.round(aurora_data['quat_x'][i], decimal_places),
            np.round(aurora_data['quat_y'][i], decimal_places),
            np.round(aurora_data['quat_z'][i], decimal_places),
            np.round(aurora_data['quat_w'][i], decimal_places),
            
            # オーロラの品質データ
            np.round(aurora_data['quality'][i], decimal_places),

            # 移動後のロボットの位置データ
            np.round(robot_after_data['x'][i], decimal_places),
            np.round(robot_after_data['y'][i], decimal_places),
            np.round(robot_after_data['z'][i], decimal_places),

            # 移動後のロボットのオイラー角データ
            np.round(robot_after_data['rx'][i], decimal_places),
            np.round(robot_after_data['ry'][i], decimal_places),
            np.round(robot_after_data['rz'][i], decimal_places),

            # 移動後のオーロラの位置データ
            np.round(aurora_after_data['x'][i], decimal_places),
            np.round(aurora_after_data['y'][i], decimal_places),
            np.round(aurora_after_data['z'][i], decimal_places),

            # 移動後のオーロラのクォータニオンデータ
            np.round(aurora_after_data['quat_x'][i], decimal_places),
            np.round(aurora_after_data['quat_y'][i], decimal_places),
            np.round(aurora_after_data['quat_z'][i], decimal_places),
            np.round(aurora_after_data['quat_w'][i], decimal_places),

            # 移動後のオーロラの品質データ
            np.round(aurora_after_data['quality'][i], decimal_places),

            # auroraの差分データ
            np.round(delta_t_aurora_data['x'][i], decimal_places),
            np.round(delta_t_aurora_data['y'][i], decimal_places),
            np.round(delta_t_aurora_data['z'][i], decimal_places),
            np.round(delta_t_aurora_data['norm'][i], decimal_places),

            # auroraの回転差分データ
            np.round(delta_R_aurora_data['rx'][i], decimal_places),
            np.round(delta_R_aurora_data['ry'][i], decimal_places),
            np.round(delta_R_aurora_data['rz'][i], decimal_places),
            np.round(delta_R_aurora_data['angle'][i], decimal_places),

            # robotの差分データ
            np.round(delta_t_robot_data['x'][i], decimal_places),
            np.round(delta_t_robot_data['y'][i], decimal_places),
            np.round(delta_t_robot_data['z'][i], decimal_places),
            np.round(delta_t_robot_data['norm'][i], decimal_places),

            # robotの回転差分データ
            np.round(delta_R_robot_data['rx'][i], decimal_places),
            np.round(delta_R_robot_data['ry'][i], decimal_places),
            np.round(delta_R_robot_data['rz'][i], decimal_places),
            np.round(delta_R_robot_data['angle'][i], decimal_places)
        ]
        data.append(row)
    
    # CSVファイルに出力
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        # ヘッダー行
        header = [
            "robot_x", "robot_y", "robot_z",
            "robot_rx", "robot_ry", "robot_rz",
            "aurora_x", "aurora_y", "aurora_z",
            "aurora_quat_x", "aurora_quat_y", "aurora_quat_z", "aurora_quat_w",
            "aurora_quality",
            "robot_after_x", "robot_after_y", "robot_after_z",
            "robot_after_rx", "robot_after_ry", "robot_after_rz",
            "aurora_after_x", "aurora_after_y", "aurora_after_z",
            "aurora_after_quat_x", "aurora_after_quat_y", "aurora_after_quat_z", "aurora_after_quat_w",
            "aurora_after_quality",
            "delta_t_aurora_x", "delta_t_aurora_y", "delta_t_aurora_z", "delta_t_aurora_norm",
            "delta_R_aurora_rx", "delta_R_aurora_ry", "delta_R_aurora_rz", "delta_R_aurora_angle",
            "delta_t_robot_x", "delta_t_robot_y", "delta_t_robot_z", "delta_t_robot_norm",
            "delta_R_robot_rx", "delta_R_robot_ry", "delta_R_robot_rz", "delta_R_robot_angle"
        ]
        writer.writerow(header)
        # データ行を書き込み
        writer.writerows(data)
    
    print(f"拡張データを {filename} に保存しました。合計 {total_points} 行。")

def cleanup(arm, aurora):
    """デバイスをクリーンアップして接続を終了"""
    aurora.close()
    arm.disconnect()
    print("デバイスの接続を終了しました")

def main(x_range, y_range, z_range, N, output_file):
    """
    メイン関数：データ収集からCSV保存までの全体の流れを制御
    
    引数:
        x_range (tuple): X座標の範囲 (開始値, 終了値)
        y_range (tuple): Y座標の範囲 (開始値, 終了値)
        z_range (tuple): Z座標の範囲 (開始値, 終了値)
        N (int): 各次元のサンプル数（N+1ポイント取得）
        output_file (str): 出力ファイルのパス
    """
    try:
        # 初期化
        arm = initialize_robot()
        
        aurora = initialize_aurora()
        
        # データ収集
        print(f"設定情報:")
        print(f"  X範囲: {x_range}")
        print(f"  Y範囲: {y_range}")
        print(f"  Z範囲: {z_range}")
        print(f"  サンプル数: {N} (各辺 {N+1} ポイント)")
        print(f"  合計測定ポイント: {(N+1)**3}")

        robot_data, aurora_data, robot_after_data, aurora_after_data = collect_diff_data(arm, aurora, x_range, y_range, z_range, N)

        delta_t_aurora_data, delta_R_aurora_data, delta_t_robot_data, delta_R_robot_data = evaluate_accuracy(robot_data, aurora_data, robot_after_data, aurora_after_data)

        # CSV保存
        save_to_csv_extended(robot_data, aurora_data, robot_after_data, aurora_after_data, delta_t_aurora_data, delta_R_aurora_data, delta_t_robot_data, delta_R_robot_data, output_file)
        
        # 正常終了
        print("処理が正常に完了しました")
        
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        # デバイスのクリーンアップ
        try:
            cleanup(arm, aurora)
        except:
            print("クリーンアップ中にエラーが発生しました")

if __name__ == "__main__":
    # ここで全てのパラメータを一か所で設定（ここだけを変更すれば良い）
    main(
        x_range=(75, 175),                    # X座標の範囲 (開始値, 終了値)
        y_range=(-50, 50),                     # Y座標の範囲 (開始値, 終了値)
        z_range=(-200, -300),                     # Z座標の範囲 (開始値, 終了値)
        N=5,                                   # サンプル数（各辺N+1ポイント）
        output_file="robot&aurora/current_code/new_transform/accuracy_test_data/transform_accuracy_20251029.csv",
    )