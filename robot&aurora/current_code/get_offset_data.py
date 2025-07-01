import time
import numpy as np
import csv
from xarm.wrapper import XArmAPI
from sksurgerynditracker.nditracker import NDITracker
from utils.pose_formatter import generateRobotArm, generateProbe

def initialize_robot(ip='192.168.1.155'):
    """ロボットアームの初期化"""
    arm = XArmAPI(ip)
    arm.connect()
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    return arm

def initialize_aurora(port='COM3'):
    """オーロラトラッカーの初期化"""
    aurora = NDITracker(
        {
            "tracker type": "aurora",
            "serial port": port,
            "use quaternions": True,
        }
    )
    aurora.start_tracking()
    time.sleep(3)  # トラッキング開始を待つ
    return aurora

def move_to_initial_pose(arm, fixed_position, initial_pose):
    """初期姿勢に移動"""
    x_fixed, y_fixed, z_fixed = fixed_position
    roll_init, pitch_init, yaw_init = initial_pose
    
    print(f"初期姿勢に移動中: roll={roll_init}°, pitch={pitch_init}°, yaw={yaw_init}°")
    
    arm.set_position(
        x=x_fixed, 
        y=y_fixed, 
        z=z_fixed,
        roll=roll_init,
        pitch=pitch_init,
        yaw=yaw_init,
        wait=True
    )
    time.sleep(3)  # 安定するまで待機

def collect_single_axis_data(arm, aurora, fixed_position, initial_pose, axis, angle_range, step_size):
    """単一軸を変化させてデータを収集"""
    x_fixed, y_fixed, z_fixed = fixed_position
    roll_init, pitch_init, yaw_init = initial_pose
    angle_start, angle_end = angle_range
    
    # 角度リストを作成（rollの場合は特別な順序）
    if axis == 'roll':
        # rollの場合：-90→-180→180→90の順序
        # -90から-180まで
        angles_negative = np.arange(angle_start, -180 - step_size, -step_size)
        # 180から90まで  
        angles_positive = np.arange(180, angle_end - step_size, -step_size)
        angles = np.concatenate([angles_negative, angles_positive])
    else:
        # pitch, yawの場合：通常の連続変化
        angles = np.arange(angle_start, angle_end + step_size, step_size)
    
    # データ格納用リスト
    data_points = []
    
    print(f"\n{axis}軸のデータ収集開始")
    if axis == 'roll':
        print(f"  角度順序: {angle_start}° → -180° → 180° → {angle_end}°（{step_size}°刻み）")
    else:
        print(f"  角度範囲: {angle_start}° から {angle_end}°（{step_size}°刻み）")
    print(f"  測定点数: {len(angles)}点")
    
    for i, angle in enumerate(angles):
        # 現在の姿勢を計算
        if axis == 'roll':
            current_roll = angle
            current_pitch = pitch_init
            current_yaw = yaw_init
        elif axis == 'pitch':
            current_roll = roll_init
            current_pitch = angle
            current_yaw = yaw_init
        elif axis == 'yaw':
            current_roll = roll_init
            current_pitch = pitch_init
            current_yaw = angle
        
        # 進捗表示
        progress = ((i + 1) / len(angles)) * 100
        print(f"  {axis}: {progress:.1f}% ({i+1}/{len(angles)}) - {angle}°")
        
        try:
            # ロボットアームを移動（角度は度単位で指定）
            arm.set_position(
                x=x_fixed, 
                y=y_fixed, 
                z=z_fixed,
                roll=current_roll,
                pitch=current_pitch,
                yaw=current_yaw,
                wait=True
            )
            time.sleep(3)
            
            # データ取得
            robot_pos = arm.get_position()
            robot = generateRobotArm(robot_pos)
            probes = generateProbe(aurora.get_frame())
            
            # データポイントを作成
            data_point = {
                'axis': axis,
                'target_angle': angle,
                'robot_x': robot.pos.x,
                'robot_y': robot.pos.y,
                'robot_z': robot.pos.z,
                'robot_roll': robot.rot.roll,
                'robot_pitch': robot.rot.pitch,
                'robot_yaw': robot.rot.yaw,
                'aurora_x': probes[0].pos.x,
                'aurora_y': probes[0].pos.y,
                'aurora_z': probes[0].pos.z,
                'aurora_quat_x': probes[0].quat.x,
                'aurora_quat_y': probes[0].quat.y,
                'aurora_quat_z': probes[0].quat.z,
                'aurora_quat_w': probes[0].quat.w,
                'aurora_quality': probes[0].quality
            }
            
            data_points.append(data_point)
            time.sleep(2)
            
        except Exception as e:
            print(f"    エラー（角度 {angle}°）: {e}")
            # エラーが発生した場合、NaNを記録
            data_point = {
                'axis': axis,
                'target_angle': angle,
                'robot_x': np.nan,
                'robot_y': np.nan,
                'robot_z': np.nan,
                'robot_roll': np.nan,
                'robot_pitch': np.nan,
                'robot_yaw': np.nan,
                'aurora_x': np.nan,
                'aurora_y': np.nan,
                'aurora_z': np.nan,
                'aurora_quat_x': np.nan,
                'aurora_quat_y': np.nan,
                'aurora_quat_z': np.nan,
                'aurora_quat_w': np.nan,
                'aurora_quality': np.nan
            }
            data_points.append(data_point)
    
    print(f"{axis}軸のデータ収集完了")
    return data_points

def collect_sequential_rotation_data(arm, aurora, fixed_position, initial_pose, roll_range, pitch_range, yaw_range, step_size):
    """各軸を順番に変化させてデータを収集"""
    all_data = []
    axes_config = [
        ('roll', roll_range),
        ('pitch', pitch_range),
        ('yaw', yaw_range)
    ]
    
    print(f"逐次回転データ収集開始")
    print(f"初期姿勢: roll={initial_pose[0]}°, pitch={initial_pose[1]}°, yaw={initial_pose[2]}°")
    print(f"固定座標: x={fixed_position[0]}, y={fixed_position[1]}, z={fixed_position[2]}")
    
    for axis, angle_range in axes_config:
        # 初期姿勢に移動
        move_to_initial_pose(arm, fixed_position, initial_pose)
        
        # 単一軸のデータを収集
        axis_data = collect_single_axis_data(
            arm, aurora, fixed_position, initial_pose, axis, angle_range, step_size
        )
        
        # 全データに追加
        all_data.extend(axis_data)
    
    # 最後に初期姿勢に戻す
    move_to_initial_pose(arm, fixed_position, initial_pose)
    
    print(f"\n全データ収集完了: 合計 {len(all_data)} 点")
    return all_data

def save_sequential_data_to_csv(data_points, filename, decimal_places=3):
    """逐次データをCSVファイルに保存"""
    
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        
        # ヘッダー行
        header = [
            "axis", "target_angle",
            "robot_x", "robot_y", "robot_z",
            "robot_roll", "robot_pitch", "robot_yaw",
            "aurora_x", "aurora_y", "aurora_z",
            "aurora_quat_x", "aurora_quat_y", "aurora_quat_z", "aurora_quat_w",
            "aurora_quality"
        ]
        writer.writerow(header)
        
        # データ行を書き込み
        for data_point in data_points:
            row = [
                data_point['axis'],
                data_point['target_angle'],
                
                # ロボットの位置データ
                np.round(data_point['robot_x'], decimal_places) if not np.isnan(data_point['robot_x']) else 'NaN',
                np.round(data_point['robot_y'], decimal_places) if not np.isnan(data_point['robot_y']) else 'NaN',
                np.round(data_point['robot_z'], decimal_places) if not np.isnan(data_point['robot_z']) else 'NaN',
                
                # ロボットのオイラー角データ
                np.round(data_point['robot_roll'], decimal_places) if not np.isnan(data_point['robot_roll']) else 'NaN',
                np.round(data_point['robot_pitch'], decimal_places) if not np.isnan(data_point['robot_pitch']) else 'NaN',
                np.round(data_point['robot_yaw'], decimal_places) if not np.isnan(data_point['robot_yaw']) else 'NaN',
                
                # オーロラの位置データ
                np.round(data_point['aurora_x'], decimal_places) if not np.isnan(data_point['aurora_x']) else 'NaN',
                np.round(data_point['aurora_y'], decimal_places) if not np.isnan(data_point['aurora_y']) else 'NaN',
                np.round(data_point['aurora_z'], decimal_places) if not np.isnan(data_point['aurora_z']) else 'NaN',
                
                # オーロラのクォータニオンデータ
                np.round(data_point['aurora_quat_x'], decimal_places) if not np.isnan(data_point['aurora_quat_x']) else 'NaN',
                np.round(data_point['aurora_quat_y'], decimal_places) if not np.isnan(data_point['aurora_quat_y']) else 'NaN',
                np.round(data_point['aurora_quat_z'], decimal_places) if not np.isnan(data_point['aurora_quat_z']) else 'NaN',
                np.round(data_point['aurora_quat_w'], decimal_places) if not np.isnan(data_point['aurora_quat_w']) else 'NaN',
                
                # オーロラの品質データ
                np.round(data_point['aurora_quality'], decimal_places) if not np.isnan(data_point['aurora_quality']) else 'NaN'
            ]
            writer.writerow(row)
    
    print(f"データを {filename} に保存しました。合計 {len(data_points)} 行。")

def cleanup(arm, aurora):
    """デバイスをクリーンアップして接続を終了"""
    aurora.close()
    arm.disconnect()
    print("デバイスの接続を終了しました")

def main_sequential_rotation(fixed_position, initial_pose, roll_range, pitch_range, yaw_range, step_size, output_file, robot_ip, aurora_port):
    """
    メイン関数：固定座標で各軸を順番に回転させてデータ収集
    
    引数:
        fixed_position (tuple): 固定座標 (x, y, z)
        initial_pose (tuple): 初期姿勢（度単位） (roll, pitch, yaw)
        roll_range (tuple): ロール角の範囲（度単位） (開始値, 終了値)
        pitch_range (tuple): ピッチ角の範囲（度単位） (開始値, 終了値)
        yaw_range (tuple): ヨー角の範囲（度単位） (開始値, 終了値)
        step_size (float): 角度のステップサイズ（度単位）
        output_file (str): 出力ファイルのパス
        robot_ip (str): ロボットアームのIPアドレス
        aurora_port (str): オーロラトラッカーのCOMポート
    """
    try:
        # 初期化
        print("ロボットアームを初期化しています...")
        arm = initialize_robot(robot_ip)
        
        print("オーロラトラッカーを初期化しています...")
        aurora = initialize_aurora(aurora_port)
        
        # 設定情報を表示
        roll_angles = np.arange(roll_range[0], roll_range[1] + step_size, step_size)
        pitch_angles = np.arange(pitch_range[0], pitch_range[1] + step_size, step_size)
        yaw_angles = np.arange(yaw_range[0], yaw_range[1] + step_size, step_size)
        
        total_points = len(roll_angles) + len(pitch_angles) + len(yaw_angles)
        
        print(f"\n設定情報:")
        print(f"  固定座標: {fixed_position}")
        print(f"  初期姿勢: roll={initial_pose[0]}°, pitch={initial_pose[1]}°, yaw={initial_pose[2]}°")
        print(f"  ロール角範囲: {roll_range[0]}° から {roll_range[1]}° ({len(roll_angles)} 点)")
        print(f"  ピッチ角範囲: {pitch_range[0]}° から {pitch_range[1]}° ({len(pitch_angles)} 点)")
        print(f"  ヨー角範囲: {yaw_range[0]}° から {yaw_range[1]}° ({len(yaw_angles)} 点)")
        print(f"  ステップサイズ: {step_size}°")
        print(f"  合計測定ポイント: {total_points} 点")
        
        # データ収集
        all_data = collect_sequential_rotation_data(
            arm, aurora, fixed_position, initial_pose, roll_range, pitch_range, yaw_range, step_size
        )
        
        # CSV保存
        save_sequential_data_to_csv(all_data, output_file)
        
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
    main_sequential_rotation(
        fixed_position=(300, 0, 125),          # 固定座標 (x, y, z)
        initial_pose=(180, 0, 0),              # 初期姿勢（度単位） (roll, pitch, yaw)
        roll_range=(-90, 90),                  # ロール角の範囲（度単位）
        pitch_range=(-60, 30),                 # ピッチ角の範囲（度単位）
        yaw_range=(-170, 180),                   # ヨー角の範囲（度単位）
        step_size=10,                          # 角度のステップサイズ（度単位）
        output_file="robot&aurora/current_code/offset_test_data/aurora_robot_sequential_rotation_log_y148.5.csv",
        robot_ip="192.168.1.155",
        aurora_port="COM3"
    )