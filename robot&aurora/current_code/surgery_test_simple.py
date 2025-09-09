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

        # upper_probeとlower_probeのAurora座標を取得
        probes = generateProbe(aurora.get_frame())
        upper_probe = probes[0]
        lower_probe = probes[1]

        # upper_probeから見たlower_probeの相対位置と姿勢を計算
        relative_pos = np.array([
            lower_probe.pos.x - upper_probe.pos.x,
            lower_probe.pos.y - upper_probe.pos.y,
            lower_probe.pos.z - upper_probe.pos.z
        ])
        time.sleep(1)

        print("相対位置の記録が完了しました。")
        print("土台とアームの位置を移動が完了したら、Enterキーを押してください。")

        # Enterキーが押された後以下の処理を実行
        input()
        print("位置を記録しています...")

        # 再度upper_probeのAurora座標を取得
        probes = generateProbe(aurora.get_frame())
        upper_probe = probes[0]

        # lower_probeのゴール位置を計算
        lower_probe_goal_pos_aurora = np.array([
            upper_probe.pos.x + relative_pos[0],
            upper_probe.pos.y + relative_pos[1],
            upper_probe.pos.z + relative_pos[2]
        ])

        # 今回は使用しないが、仮としてlower_probeのクォータニオンをそのまま使用
        lower_probe_goal_quat_aurora = np.array([
            lower_probe.quat.w,
            lower_probe.quat.x,
            lower_probe.quat.y,
            lower_probe.quat.z
        ])

        sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = adaptive_transform.main(
            x_range=(100, 150),                    # X coordinate range (start_value, end_value)
            y_range=(-25, 25),                     # Y coordinate range (start_value, end_value)
            z_range=(-325, -275),                     # Z coordinate range (start_value, end_value)
            divisions=1,                           # Number of divisions (divisions per axis for divisions^3 regions)
            data_file='robot&aurora/current_code/calibration_data/aurora_robot_pose_log.csv',  # Data file path
            input_point=lower_probe_goal_pos_aurora,             # Coordinates of point to transform [x, y, z]
            input_quaternion=lower_probe_goal_quat_aurora          # Quaternion representing pose [x, y, z, w]
        )

        lower_probe_goal_pos_robot = sensor_point_from_robot

        # 現在のロボットアームの位置を取得
        robot = generateRobotArm(arm.get_position())
        current_arm_pos_robot = np.array([robot.pos.x, robot.pos.y, robot.pos.z])

        # ロボットアームをまずは平面上(y,z)のみゴール位置に移動
        arm.set_position(x=current_arm_pos_robot[0], y=lower_probe_goal_pos_robot[1], wait=True)

        # z軸方向の位置を移動
        arm.set_position(x=current_arm_pos_robot[0], y=lower_probe_goal_pos_robot[1], z=lower_probe_goal_pos_robot[2], wait=True)
        print("ロボットアームの移動が完了しました。")

    except Exception as e:
        print(f"エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()