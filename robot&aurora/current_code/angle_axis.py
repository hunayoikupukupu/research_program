import time
from datetime import datetime
import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    # Auroraトラッカーのライブラリをインポート
    from sksurgerynditracker.nditracker import NDITracker
    from utils.pose_formatter import generateRobotArm, generateRobotArmAxisAngle, generateProbe
    from utils.initialization import initialize_robot, initialize_aurora
except ImportError as e:
    print(f"エラー: 必要なライブラリがインポートできません。{e}")
    print("utils.probe モジュールと sksurgerynditracker がインストールされていることを確認してください。")
    sys.exit(1)

def main():
    # ポート設定
    port = "COM3"  # 環境に合わせて変更してください
    
    print("=" * 50)
    print(f"Aurora トラッカー 姿勢情報表示 (ポート: {port})")
    print("=" * 50)
    print("Ctrl+C で終了")
    print("初期化中...")
    
    try:
        # ロボットアームを初期化
        arm = initialize_robot()
        # オーロラトラッカーを初期化
        aurora = initialize_aurora(port=port)

        print("トラッキング開始しました")
        print("-" * 50)
        
        # メインループ
        try:
            while True:
                # 現在時刻取得
                now = datetime.now()
                timestamp = now.strftime("%H:%M:%S.%f")[:-3]

                # Auroraのプローブからデータ取得
                probes = generateProbe(aurora.get_frame())
                
                if not probes:
                    print("プローブが検出されません。センサーの接続を確認してください。")
                    time.sleep(1)
                    continue
                
                # 最初のプローブデータを使用
                probe = probes[0]
    
                pos_x, pos_y, pos_z = probe.pos.x, probe.pos.y, probe.pos.z
                quat_w, quat_x, quat_y, quat_z = probe.quat.w, probe.quat.x, probe.quat.y, probe.quat.z
                quality = probe.quality
                
                # クォータニオンからオイラー角に変換（単位：度）
                try:
                    # scipyの関数を使用してオイラー角に変換
                    rotation = R.from_quat([quat_x, quat_y, quat_z, quat_w])
                    euler_angles = rotation.as_euler('zyx', degrees=True)
                    roll, pitch, yaw = euler_angles
                    euler_str = f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°"
                except Exception as e:
                    # エラーが発生した場合はnanを表示
                    print(f"警告: オイラー角変換エラー [{quat_x:.4f}, {quat_y:.4f}, {quat_z:.4f}, {quat_w:.4f}] - {str(e)}")
                    euler_str = "Roll=nan, Pitch=nan, Yaw=nan"

                # ロボットアームの位置と姿勢を取得（オイラー角）
                robot = generateRobotArm(arm.get_position())
                robot_pos_x, robot_pos_y, robot_pos_z = robot.pos.x, robot.pos.y, robot.pos.z
                robot_roll, robot_pitch, robot_yaw = robot.rot.roll, robot.rot.pitch, robot.rot.yaw

                # ロボットアームの位置と姿勢を取得（AxisAngle）
                robot_axis_angle = generateRobotArmAxisAngle(arm.get_position_aa())
                robot_rx, robot_ry, robot_rz = robot_axis_angle.rot.rx, robot_axis_angle.rot.ry, robot_axis_angle.rot.rz

                try:
                    # AxisAngleからクォータニオンに変換（単位：度）
                    rotation = R.from_rotvec([robot_rx, robot_ry, robot_rz], degrees=True)
                    robot_quat = rotation.as_quat()
                    robot_quat_w, robot_quat_x, robot_quat_y, robot_quat_z = robot_quat[3], robot_quat[0], robot_quat[1], robot_quat[2]
                except Exception as e:
                    # エラーが発生した場合はnanを表示
                    print(f"警告: AxisAngleからクォータニオン変換エラー [{robot_rx:.4f}, {robot_ry:.4f}, {robot_rz:.4f}] - {str(e)}")
                    robot_quat_w, robot_quat_x, robot_quat_y, robot_quat_z = float('nan'), float('nan'), float('nan'), float('nan')


                # データ表示
                print(f"[{timestamp}]")
                print(f"Aurora位置: X={pos_x:.2f}, Y={pos_y:.2f}, Z={pos_z:.2f}")
                print(f"クォータニオン: W={quat_w:.4f}, X={quat_x:.4f}, Y={quat_y:.4f}, Z={quat_z:.4f}")
                print(f"オイラー角 (zyx): {euler_str}")
                print(f"クオリティ: {quality:.2f}")
                print(f"ロボットアーム位置: X={robot_pos_x:.2f}, Y={robot_pos_y:.2f}, Z={robot_pos_z:.2f}")
                print(f"ロボットアーム姿勢 (オイラー角): Roll={robot_roll:.2f}°, Pitch={robot_pitch:.2f}°, Yaw={robot_yaw:.2f}°")
                print(f"ロボットアーム姿勢 (AxisAngle): RX={robot_rx:.2f}, RY={robot_ry:.2f}, RZ={robot_rz:.2f}")
                print(f"ロボットアームクォータニオン: W={robot_quat_w:.4f}, X={robot_quat_x:.4f}, Y={robot_quat_y:.4f}, Z={robot_quat_z:.4f}")
                print("-" * 50)


                
                # 1秒待機
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nプログラムを終了します...")
        finally:
            # トラッキング停止
            aurora.stop_tracking()
            print("トラッキングを停止しました")
    
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()