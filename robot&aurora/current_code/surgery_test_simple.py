import sys
import numpy as np
import time
import adaptive_transform

try:
    # Auroraトラッカーのライブラリをインポート
    from utils.pose_formatter import generateRobotArm, generateRobotArmAxisAngle, generateProbe
    from utils.initialization import initialize_robot, initialize_aurora
except ImportError as e:
    print(f"エラー: 必要なライブラリがインポートできません。{e}")
    print("utils.probe モジュールと sksurgerynditracker がインストールされていることを確認してください。")
    sys.exit(1)

def record_relative_position(aurora):
    """相対位置を記録する関数"""
    print("相対位置を記録中...")
    
    # upper_probeとlower_probeのAurora座標を取得
    probes = generateProbe(aurora.get_frame())
    upper_probe = probes[1]
    lower_probe = probes[0]

    # upper_probeから見たlower_probeの相対位置を計算
    relative_pos = np.array([
        lower_probe.pos.x - upper_probe.pos.x,
        lower_probe.pos.y - upper_probe.pos.y,
        lower_probe.pos.z - upper_probe.pos.z
    ])
    
    # lower_probeのクォータニオンを記録
    lower_probe_quat = np.array([
        lower_probe.quat.w,
        lower_probe.quat.x,
        lower_probe.quat.y,
        lower_probe.quat.z
    ])
    
    print("相対位置の記録が完了しました。")
    return relative_pos, lower_probe_quat

def move_robot_to_goal(arm, aurora, relative_pos, lower_probe_quat):
    """ロボットをゴール位置に移動する関数"""
    print("ゴール位置を計算中...")
    
    # 現在のupper_probeのAurora座標を取得
    probes = generateProbe(aurora.get_frame())
    upper_probe = probes[1]

    # lower_probeのゴール位置を計算
    lower_probe_goal_pos_aurora = np.array([
        upper_probe.pos.x + relative_pos[0],
        upper_probe.pos.y + relative_pos[1],
        upper_probe.pos.z + relative_pos[2]
    ])

    # adaptive_transformで座標変換
    sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = adaptive_transform.main(
        x_range=(50, 200),
        y_range=(-75, 75),
        z_range=(-400, -250),
        divisions=1,
        data_file='robot&aurora/current_code/calibration_data/aurora_robot_pose_log_6_6_6.csv',
        input_point=lower_probe_goal_pos_aurora,
        input_quaternion=lower_probe_quat
    )

    lower_probe_goal_pos_robot = sensor_point_from_robot

    print(f"ゴール位置 (Robot座標): {lower_probe_goal_pos_robot}")
    
    # ロボットアームを段階的に移動
    print("ロボットアームをy,z方向に移動中...")
    arm.set_position(y=lower_probe_goal_pos_robot[1], z=lower_probe_goal_pos_robot[2], wait=True, speed=50)
    
    print("1秒後にx軸方向の移動を開始します...")
    time.sleep(1)
    
    print("ロボットアームをx方向に移動中...")
    arm.set_position(x=lower_probe_goal_pos_robot[0], wait=True, speed=50)
    
    print("ロボットアームの移動が完了しました。")

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
        print("=" * 50)
        
        # 初回のみ相対位置を記録
        print("\n--- 初期設定: 相対位置の記録 ---")
        relative_pos, lower_probe_quat = record_relative_position(aurora)
        print("相対位置が記録されました。この位置関係を維持してロボット制御を行います。")
        
        cycle_count = 1
        
        while True:
            try:
                print(f"\n--- サイクル {cycle_count} ---")
                
                # ステップ1: ユーザーの操作待ち
                print("土台とアームの位置を移動が完了したら、Enterキーを押してください。")
                print("(終了する場合は Ctrl+C を押してください)")
                input(">>> Enterキーを押してください: ")
                
                # ステップ2: ロボットをゴール位置に移動（記録済みの相対位置を使用）
                move_robot_to_goal(arm, aurora, relative_pos, lower_probe_quat)
                
                print(f"\nサイクル {cycle_count} が完了しました。")
                print("次のサイクルを開始します...")
                
                cycle_count += 1
                time.sleep(2)  # 次のサイクルまで少し待機
                
            except KeyboardInterrupt:
                # 内側のループでCtrl+Cが押された場合
                raise
                
    except KeyboardInterrupt:
        print("\n\nCtrl+C が押されました。プログラムを終了します...")
        print("お疲れさまでした！")
        
    except Exception as e:
        print(f"\nエラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # クリーンアップ処理があれば、ここに記述
        print("プログラムを終了しました。")

if __name__ == "__main__":
    main()