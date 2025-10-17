import sys
import numpy as np
import time
import adaptive_transform
from scipy.spatial.transform import Rotation as R

try:
    # Auroraトラッカーのライブラリをインポート
    from utils.pose_formatter import generateRobotArm, generateRobotArmAxisAngle, generateProbe
    from utils.initialization import initialize_robot, initialize_aurora
except ImportError as e:
    print(f"エラー: 必要なライブラリがインポートできません。{e}")
    print("utils.probe モジュールと sksurgerynditracker がインストールされていることを確認してください。")
    sys.exit(1)

def pose_to_matrix(pos, quat):
    """位置ベクトルとクォータニオンから4x4同次変換行列を作成する関数"""
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat(quat).as_matrix()
    matrix[:3, 3] = pos
    return matrix

def record_relative_transform(aurora):
    """2つのプローブ間の相対的な剛体変換行列を記録する関数"""
    print("相対的な変換行列を記録中...")
    
    # upper_probeとlower_probeのAurora座標を取得
    probes = generateProbe(aurora.get_frame())
    upper_probe = probes[1]
    lower_probe = probes[0]

    # 各プローブの位置と姿勢を抽出
    upper_pos = np.array([upper_probe.pos.x, upper_probe.pos.y, upper_probe.pos.z])
    upper_quat = np.array([upper_probe.quat.x, upper_probe.quat.y, upper_probe.quat.z, upper_probe.quat.w])
    
    lower_pos = np.array([lower_probe.pos.x, lower_probe.pos.y, lower_probe.pos.z])
    lower_quat = np.array([lower_probe.quat.x, lower_probe.quat.y, lower_probe.quat.z, lower_probe.quat.w])

    # ワールド座標系（Aurora）から各プローブへの変換行列を計算
    T_world_upper = pose_to_matrix(upper_pos, upper_quat)
    T_world_lower = pose_to_matrix(lower_pos, lower_quat)
    
    # upperプローブから見たlowerプローブの相対変換行列を計算
    # T_upper_to_lower = inv(T_world_to_upper) @ T_world_to_lower
    T_upper_lower = np.linalg.inv(T_world_upper) @ T_world_lower

    print("相対的な変換行列の記録が完了しました。")
    return T_upper_lower

def move_robot_to_goal(arm, aurora, T_upper_lower):
    """ロボットをゴール位置に移動する関数"""
    print("ゴール位置を計算中...")
    
    # 現在のupper_probeのAurora座標を取得
    probes = generateProbe(aurora.get_frame())
    upper_probe = probes[1]

    # 現在のupper_probeの位置と姿勢から、ワールド座標系での変換行列を計算
    current_upper_pos = np.array([upper_probe.pos.x, upper_probe.pos.y, upper_probe.pos.z])
    current_upper_quat = np.array([upper_probe.quat.x, upper_probe.quat.y, upper_probe.quat.z, upper_probe.quat.w])
    T_world_upper_current = pose_to_matrix(current_upper_pos, current_upper_quat)

    # lower_probeのゴールとなる変換行列を計算
    # T_world_to_lower_goal = T_world_to_upper_current @ T_upper_to_lower
    T_world_lower_goal = T_world_upper_current @ T_upper_lower

    # ゴール変換行列から位置とクォータニオンを抽出
    lower_probe_goal_pos_aurora = T_world_lower_goal[:3, 3]
    lower_probe_goal_rot_matrix = T_world_lower_goal[:3, :3]
    lower_probe_goal_quat_aurora = R.from_matrix(lower_probe_goal_rot_matrix).as_quat()

    # adaptive_transformで座標変換
    sensor_point_from_robot, sensor_R_vector_from_robot, sensor_quat_from_robot, arm_R_vector_from_robot, arm_quat_from_robot = adaptive_transform.main(
        x_range=(50, 200),
        y_range=(-75, 75),
        z_range=(-350, -200),
        divisions=1,
        data_file='robot&aurora/current_code/calibration_data/aurora_robot_pose_log_20251009.csv',
        input_point=lower_probe_goal_pos_aurora,
        input_quaternion=lower_probe_goal_quat_aurora
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

    angle_pose = [lower_probe_goal_pos_robot[0], lower_probe_goal_pos_robot[1], lower_probe_goal_pos_robot[2], arm_R_vector_from_robot[0], arm_R_vector_from_robot[1], arm_R_vector_from_robot[2]]
    print("ロボットアームの姿勢を調整中...")
    arm.set_position_aa(axis_angle_pose=angle_pose, wait=True, speed=20)

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
        
        # 初回のみ相対変換行列を記録
        print("\n--- 初期設定: 相対的な変換行列の記録 ---")
        relative_transform = record_relative_transform(aurora)
        print("相対的な変換行列が記録されました。この関係を維持してロボット制御を行います。")
        
        cycle_count = 1
        
        while True:
            try:
                print(f"\n--- サイクル {cycle_count} ---")
                
                # ステップ1: ユーザーの操作待ち
                print("土台とアームの位置を移動が完了したら、Enterキーを押してください。")
                print("(終了する場合は Ctrl+C を押してください)")
                input(">>> Enterキーを押してください: ")
                
                # ステップ2: ロボットをゴール位置に移動（記録済みの相対変換行列を使用）
                move_robot_to_goal(arm, aurora, relative_transform)
                
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