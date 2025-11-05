import time
import numpy as np
import csv
from utils.pose_formatter import generateRobotArmAxisAngle, generateProbe
from utils.initialization import initialize_robot, initialize_aurora

# --- 変更点: 新しいデータ収集関数 ---
def collect_data_by_orientation(arm, aurora, fixed_pos, roll_range, pitch_range, yaw_ranges, N):
    """
    指定された固定座標で、姿勢（roll, pitch, yaw）を変化させながらデータを収集する
    """
    # 各角度の範囲とステップ数を設定
    roll_start, roll_end = roll_range
    pitch_start, pitch_end = pitch_range

    # numpy.linspaceを使い、各軸で (N+1) 個のサンプリングポイントを生成
    roll_values = np.linspace(roll_start, roll_end, N + 1)
    pitch_values = np.linspace(pitch_start, pitch_end, N + 1)
    
    # yawは範囲が2つあるため、それぞれでサンプリングポイントを生成し、結合する
    yaw_values_1 = np.linspace(yaw_ranges[0][0], yaw_ranges[0][1], N + 1)
    yaw_values_2 = np.linspace(yaw_ranges[1][0], yaw_ranges[1][1], N + 1)
    all_yaw_values = np.concatenate((yaw_values_1, yaw_values_2))

    # 全体のデータポイント数を計算
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
    
    print(f"データ収集開始: 合計 {total_points} ポイント")
    
    point_count = 0
    # 各姿勢での計測
    for roll in roll_values:
        for pitch in pitch_values:
            for yaw in all_yaw_values:
                # 進捗表示
                if point_count % 10 == 0:
                    progress = (point_count / total_points) * 100
                    print(f"進捗: {progress:.1f}% ({point_count}/{total_points})")
                
                # --- ★★★ ここが重要な変更点 ★★★ ---
                # 固定されたXYZ座標と、ループで変化するroll, pitch, yawを使ってアームを移動
                arm.set_position(
                    x=fixed_pos['x'], y=fixed_pos['y'], z=fixed_pos['z'], 
                    roll=roll, pitch=pitch, yaw=yaw, 
                    speed=50, wait=True
                )
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
                point_count += 1
    
    print("データ収集完了")
    return robot_data, aurora_data


def save_to_csv_extended(robot_data, aurora_data, filename, decimal_places=3):
    """拡張データをCSVファイルに保存（この関数は変更なし）"""
    data = []
    total_points = len(robot_data['x'])
    
    for i in range(total_points):
        row = [
            np.round(robot_data['x'][i], decimal_places),
            np.round(robot_data['y'][i], decimal_places),
            np.round(robot_data['z'][i], decimal_places),
            np.round(robot_data['rx'][i], decimal_places),
            np.round(robot_data['ry'][i], decimal_places),
            np.round(robot_data['rz'][i], decimal_places),
            np.round(aurora_data['x'][i], decimal_places),
            np.round(aurora_data['y'][i], decimal_places),
            np.round(aurora_data['z'][i], decimal_places),
            np.round(aurora_data['quat_x'][i], decimal_places),
            np.round(aurora_data['quat_y'][i], decimal_places),
            np.round(aurora_data['quat_z'][i], decimal_places),
            np.round(aurora_data['quat_w'][i], decimal_places),
            np.round(aurora_data['quality'][i], decimal_places)
        ]
        data.append(row)
    
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        header = [
            "robot_x", "robot_y", "robot_z",
            "robot_rx", "robot_ry", "robot_rz",
            "aurora_x", "aurora_y", "aurora_z",
            "aurora_quat_x", "aurora_quat_y", "aurora_quat_z", "aurora_quat_w",
            "aurora_quality"
        ]
        writer.writerow(header)
        writer.writerows(data)
    
    print(f"拡張データを {filename} に保存しました。合計 {total_points} 行。")


def cleanup(arm, aurora):
    """デバイスをクリーンアップして接続を終了（この関数は変更なし）"""
    aurora.close()
    arm.disconnect()
    print("デバイスの接続を終了しました")


# --- 変更点: main関数のパラメータ設定 ---
def main(fixed_pos, roll_range, pitch_range, yaw_ranges, N, output_file):
    """
    メイン関数：姿勢を変化させるデータ収集からCSV保存までの流れを制御
    """
    try:
        arm = initialize_robot()
        aurora = initialize_aurora()
        
        # --- 変更点: 設定情報の表示を更新 ---
        print(f"設定情報:")
        print(f"  固定座標: {fixed_pos}")
        print(f"  Roll範囲: {roll_range}")
        print(f"  Pitch範囲: {pitch_range}")
        print(f"  Yaw範囲: {yaw_ranges}")
        print(f"  各軸のサンプル数: {N} (各範囲で {N+1} ポイント)")
        
        # 合計ポイント数の計算
        num_yaw_points = (N + 1) * len(yaw_ranges)
        total_points = (N + 1) * (N + 1) * num_yaw_points
        print(f"  合計測定ポイント: {total_points}")

        # --- 変更点: 新しいデータ収集関数を呼び出し ---
        robot_data, aurora_data = collect_data_by_orientation(
            arm, aurora, fixed_pos, roll_range, pitch_range, yaw_ranges, N
        )

        save_to_csv_extended(robot_data, aurora_data, output_file)
        
        print("処理が正常に完了しました")
        
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        try:
            cleanup(arm, aurora)
        except:
            print("クリーンアップ中にエラーが発生しました")


if __name__ == "__main__":
    # --- ★★★ ここで全てのパラメータを一か所で設定 ★★★ ---
    main(
        # --- 変更点: パラメータ設定 ---
        fixed_pos={'x': 150, 'y': 0, 'z': -200}, # ロボットアームを固定するXYZ座標
        
        roll_range=(-30, 30),                     # Rollの範囲 (開始角度, 終了角度)
        pitch_range=(-30, 30),                    # Pitchの範囲 (開始角度, 終了角度)
        
        # Yawの範囲（複数の範囲をリストで指定）
        yaw_ranges=[(-180, -150), (150, 180)], 
        
        N=2,                                      # 各範囲のサンプル数（各範囲でN+1ポイント取得）
        
        output_file="robot&aurora/current_code/new_transform/data/aurora_robot_orientation_log_2020510281902.csv"
    )