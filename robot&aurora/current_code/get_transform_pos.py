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

def collect_data(arm, aurora, x_range, y_range, z_range, N):
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
        'roll': [], 'pitch': [], 'yaw': []
    }
    
    aurora_data = {
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
        arm.set_position(x=current_x, y=current_y, z=current_z, wait=True)
        time.sleep(1)
        
        # データ取得
        robot = generateRobotArm(arm.get_position())
        probes = generateProbe(aurora.get_frame())
        
        # ロボットデータを保存
        robot_data['x'].append(robot.pos.x)
        robot_data['y'].append(robot.pos.y)
        robot_data['z'].append(robot.pos.z)
        robot_data['roll'].append(robot.rot.roll)
        robot_data['pitch'].append(robot.rot.pitch)
        robot_data['yaw'].append(robot.rot.yaw)
        
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
    
    print("データ収集完了")
    return robot_data, aurora_data

def process_data(robot_data, aurora_data):
    """収集したデータを処理してnumpy配列に変換"""
    # ロボットの位置データ
    P1 = np.column_stack((
        robot_data['x'], 
        robot_data['y'], 
        robot_data['z']
    ))
    
    # オーロラの位置とクオリティデータ
    P2 = np.column_stack((
        aurora_data['x'], 
        aurora_data['y'], 
        aurora_data['z'], 
        aurora_data['quality']
    ))
    
    # ロボットの回転データ
    robot_rotation = np.column_stack((
        robot_data['roll'],
        robot_data['pitch'],
        robot_data['yaw']
    ))
    
    # オーロラのクォータニオンデータ
    aurora_quaternion = np.column_stack((
        aurora_data['quat_x'],
        aurora_data['quat_y'],
        aurora_data['quat_z'],
        aurora_data['quat_w']
    ))
    
    return P1, P2, robot_rotation, aurora_quaternion

def save_to_csv_extended(robot_data, aurora_data, filename, decimal_places=3):
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
            np.round(robot_data['roll'][i], decimal_places),
            np.round(robot_data['pitch'][i], decimal_places),
            np.round(robot_data['yaw'][i], decimal_places),
            
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
            np.round(aurora_data['quality'][i], decimal_places)
        ]
        data.append(row)
    
    # CSVファイルに出力
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        # ヘッダー行
        header = [
            "robot_x", "robot_y", "robot_z",
            "robot_roll", "robot_pitch", "robot_yaw",
            "aurora_x", "aurora_y", "aurora_z",
            "aurora_quat_x", "aurora_quat_y", "aurora_quat_z", "aurora_quat_w",
            "aurora_quality"
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

def main(x_range, y_range, z_range, N, output_file, robot_ip, aurora_port):
    """
    メイン関数：データ収集からCSV保存までの全体の流れを制御
    
    引数:
        x_range (tuple): X座標の範囲 (開始値, 終了値)
        y_range (tuple): Y座標の範囲 (開始値, 終了値)
        z_range (tuple): Z座標の範囲 (開始値, 終了値)
        N (int): 各次元のサンプル数（N+1ポイント取得）
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
        
        # データ収集
        print(f"設定情報:")
        print(f"  X範囲: {x_range}")
        print(f"  Y範囲: {y_range}")
        print(f"  Z範囲: {z_range}")
        print(f"  サンプル数: {N} (各辺 {N+1} ポイント)")
        print(f"  合計測定ポイント: {(N+1)**3}")
        
        robot_data, aurora_data = collect_data(arm, aurora, x_range, y_range, z_range, N)
        
        # データ処理
        P1, P2, robot_rotation, aurora_quaternion = process_data(robot_data, aurora_data)
        
        # CSV保存
        save_to_csv_extended(robot_data, aurora_data, output_file)
        
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
        x_range=(275, 325),                    # X座標の範囲 (開始値, 終了値)
        y_range=(-25, 25),                     # Y座標の範囲 (開始値, 終了値)
        z_range=(100, 150),                     # Z座標の範囲 (開始値, 終了値)
        N=2,                                   # サンプル数（各辺N+1ポイント）
        output_file="robot&aurora/current_code/calibration_data/aurora_robot_pose_log_t.csv",
        robot_ip="192.168.1.155",
        aurora_port="COM3"
    )
    