# 座標・姿勢変換の正確性を評価するためのデータ収集スクリプト
import time
import numpy as np
import csv
from utils.pose_formatter import generateRobotArmAxisAngle, generateProbe
from utils.initialization import initialize_robot, initialize_aurora
from scipy.spatial.transform import Rotation as R
from calibration.transformation_utils import Transform, compute_transform_difference
from main import main as run_pose_transoformation


def collect_diff_data_by_orientation(arm, aurora, position, roll_range, pitch_range, yaw_ranges, N):
    """
    指定された固定位置で、姿勢（roll, pitch, yaw）を変化させながらデータを収集する
    """
    # 固定位置を設定
    fixed_x, fixed_y, fixed_z = position
    
    # 各角度で生成する値のリストを作成
    roll_values = np.linspace(roll_range[0], roll_range[1], N + 1)
    pitch_values = np.linspace(pitch_range[0], pitch_range[1], N + 1)
    
    # Yawは2つの範囲から値を生成し、結合する
    yaw_values_1 = np.linspace(yaw_ranges[0][0], yaw_ranges[0][1], N + 1)
    yaw_values_2 = np.linspace(yaw_ranges[1][0], yaw_ranges[1][1], N + 1)
    all_yaw_values = np.concatenate((yaw_values_1, yaw_values_2))
    
    # 合計測定ポイント数を計算
    total_points = len(roll_values) * len(pitch_values) * len(all_yaw_values)
    
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
    
    point_counter = 0
    # 各姿勢での計測
    for current_roll in roll_values:
        for current_pitch in pitch_values:
            for current_yaw in all_yaw_values:
                
                # 進捗表示
                if point_counter % 10 == 0:
                    progress = (point_counter / total_points) * 100
                    print(f"進捗: {progress:.1f}% ({point_counter}/{total_points})")
                
                # ロボットアームを移動（位置を固定し、姿勢を変化させる）
                arm.set_position(
                    x=fixed_x, y=fixed_y, z=fixed_z,
                    roll=current_roll, pitch=current_pitch, yaw=current_yaw,
                    speed=50, wait=True
                )
                time.sleep(2)
                
                # --- ここから下は元のスクリプトのループ内ロジック ---
                
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
                # --- ここまでが元のスクリプトのループ内ロジック ---

                point_counter += 1
    
    print("データ収集完了")
    return robot_data, aurora_data, robot_after_data, aurora_after_data

# ===================================================================
# 以下の関数 (evaluate_accuracy, save_to_csv_extended) は
# データの収集方法に依存しないため、変更ありません。
# ===================================================================

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
    if aurora:
        aurora.close()
    if arm:
        arm.disconnect()
    print("デバイスの接続を終了しました")

def main(position, roll_range, pitch_range, yaw_ranges, N, output_file):
    """
    メイン関数：データ収集からCSV保存までの全体の流れを制御
    
    引数:
        position (tuple): X, Y, Z座標の固定値
        roll_range (tuple): Rollの範囲 (開始角度, 終了角度)
        pitch_range (tuple): Pitchの範囲 (開始角度, 終了角度)
        yaw_ranges (tuple): Yawの範囲のタプル ((範囲1開始, 範囲1終了), (範囲2開始, 範囲2終了))
        N (int): 各角度範囲のサンプル分割数（N+1ポイント取得）
        output_file (str): 出力ファイルのパス
    """
    arm = None
    aurora = None
    try:
        # 初期化
        arm = initialize_robot()
        aurora = initialize_aurora()
        
        # 設定情報を表示
        print("設定情報:")
        print(f"  固定位置 (X, Y, Z): {position}")
        print(f"  Roll範囲: {roll_range}")
        print(f"  Pitch範囲: {pitch_range}")
        print(f"  Yaw範囲: {yaw_ranges[0]} および {yaw_ranges[1]}")
        print(f"  各範囲のサンプル分割数: {N} ({N+1} ポイント)")

        # データ収集（★★ 関数名を変更 ★★）
        robot_data, aurora_data, robot_after_data, aurora_after_data = collect_diff_data_by_orientation(
            arm, aurora, position, roll_range, pitch_range, yaw_ranges, N
        )

        # 正確性評価
        delta_t_aurora_data, delta_R_aurora_data, delta_t_robot_data, delta_R_robot_data = evaluate_accuracy(
            robot_data, aurora_data, robot_after_data, aurora_after_data
        )

        # CSV保存
        save_to_csv_extended(
            robot_data, aurora_data, robot_after_data, aurora_after_data, 
            delta_t_aurora_data, delta_R_aurora_data, delta_t_robot_data, delta_R_robot_data, 
            output_file
        )
        
        print("処理が正常に完了しました")
        
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        # デバイスのクリーンアップ
        try:
            if arm or aurora:
                cleanup(arm, aurora)
            else:
                print("デバイスが初期化されなかったため、クリーンアップをスキップします")
        except Exception as e:
            print(f"クリーンアップ中にエラーが発生しました: {e}")

if __name__ == "__main__":
    # ===== パラメータ設定 =====
    # ここで全てのパラメータを一か所で設定します（ここだけを変更すれば良い）
    
    # 1. ロボットアームを固定する座標 (x, y, z)
    # 元のプログラムの範囲の中心あたりを例として設定しています。
    FIXED_POSITION = (150, 0, -225)
    
    # 2. 変化させる姿勢の範囲
    ROLL_RANGE = (-30, 30)                # Rollの範囲 (開始角度, 終了角度)
    PITCH_RANGE = (-30, 30)               # Pitchの範囲 (開始角度, 終了角度)
    YAW_RANGES = ((-180, -150), (150, 180)) # Yawの2つの範囲
    
    # 3. 各角度範囲の分割数（この数+1個のポイントが生成されます）
    N_SAMPLES = 5
    
    # 4. 出力するCSVファイル名 (ファイル名を変更推奨)
    OUTPUT_FILE = "robot&aurora/current_code/new_transform/accuracy_test_data/transform_accuracy_orientation_202510290053.csv"
    
    # ===== プログラム実行 =====
    main(
        position=FIXED_POSITION,
        roll_range=ROLL_RANGE,
        pitch_range=PITCH_RANGE,
        yaw_ranges=YAW_RANGES,
        N=N_SAMPLES,
        output_file=OUTPUT_FILE,
    )