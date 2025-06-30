import time
from datetime import datetime
import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

# 必要なパスを追加
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

try:
    # Auroraトラッカーのライブラリをインポート
    from sksurgerynditracker.nditracker import NDITracker
    from utils.pose_formatter import generateRobotArm, generateProbe
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
        # Auroraトラッカーを初期化
        aurora = NDITracker({
            "tracker type": "aurora",
            "serial port": port,
            "use quaternions": True,
        })
        
        # トラッキング開始
        time.sleep(1)
        aurora.start_tracking()
        time.sleep(1)
        
        print("トラッキング開始しました")
        print("-" * 50)
        
        # メインループ
        try:
            while True:
                # フレーム取得
                frame_data = aurora.get_frame()
                
                # プローブデータ取得
                probes = generateProbe(aurora.get_frame())
                
                if not probes:
                    print("プローブが検出されません。センサーの接続を確認してください。")
                    time.sleep(1)
                    continue
                
                # 現在時刻取得
                now = datetime.now()
                timestamp = now.strftime("%H:%M:%S.%f")[:-3]
                
                # 最初のプローブデータを使用
                probe = probes[0]
                

                # 位置とクォータニオンを取得
                pos_x = probe.pos.x
                pos_y = probe.pos.y
                pos_z = probe.pos.z
                
                quat_w = probe.quat.w
                quat_x = probe.quat.x
                quat_y = probe.quat.y
                quat_z = probe.quat.z

                quality = probe.quality
                
                # クォータニオンからオイラー角に変換（単位：度）
                try:
                    # scipyの関数を使用してオイラー角に変換
                    rotation = R.from_quat([quat_x, quat_y, quat_z, quat_w])
                    euler_angles = rotation.as_euler('xyz', degrees=True)
                    roll, pitch, yaw = euler_angles
                    euler_str = f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°"
                except Exception as e:
                    # エラーが発生した場合はnanを表示
                    print(f"警告: オイラー角変換エラー [{quat_x:.4f}, {quat_y:.4f}, {quat_z:.4f}, {quat_w:.4f}] - {str(e)}")
                    euler_str = "Roll=nan, Pitch=nan, Yaw=nan"
                
                # データ表示
                print(f"[{timestamp}]")
                print(f"位置: X={pos_x:.2f}, Y={pos_y:.2f}, Z={pos_z:.2f}")
                print(f"クォータニオン: W={quat_w:.4f}, X={quat_x:.4f}, Y={quat_y:.4f}, Z={quat_z:.4f}")
                print(f"オイラー角 (XYZ): {euler_str}")
                print(f"クオリティ: {quality:.2f}")
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