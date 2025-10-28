import sys
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from calibration.transformation_utils import Transform, compute_transform_difference
from main import main as run_pose_transoformation

try:
    # Auroraトラッカーのライブラリをインポート
    from utils.pose_formatter import generateRobotArm, generateRobotArmAxisAngle, generateProbe
    from utils.initialization import initialize_robot, initialize_aurora
except ImportError as e:
    print(f"エラー: 必要なライブラリがインポートできません。{e}")
    print("utils.probe モジュールと sksurgerynditracker がインストールされていることを確認してください。")
    sys.exit(1)

def record_relative_transform(aurora):
    """2つのプローブ間の相対的な剛体変換行列を記録する関数"""
    print("相対的な変換行列を記録中...")
    
    # upper_probeとlower_probeのAurora座標を取得
    probes = generateProbe(aurora.get_frame())
    upper_probe = probes[1]
    lower_probe = probes[0]

    # 各プローブの位置と姿勢を抽出
    t_upper_from_aurora = np.array([upper_probe.pos.x, upper_probe.pos.y, upper_probe.pos.z])
    quat_upper_from_aurora = np.array([upper_probe.quat.x, upper_probe.quat.y, upper_probe.quat.z, upper_probe.quat.w])
    R_upper_from_aurora = R.from_quat(quat_upper_from_aurora).as_matrix()
    T_upper_from_aurora_transform = Transform(R_upper_from_aurora, t_upper_from_aurora)

    t_lower_from_aurora = np.array([lower_probe.pos.x, lower_probe.pos.y, lower_probe.pos.z])
    quat_lower_from_aurora = np.array([lower_probe.quat.x, lower_probe.quat.y, lower_probe.quat.z, lower_probe.quat.w])
    R_lower_from_aurora = R.from_quat(quat_lower_from_aurora).as_matrix()
    T_lower_from_aurora_transform = Transform(R_lower_from_aurora, t_lower_from_aurora)

    # upperプローブから見たlowerプローブの相対変換行列を計算
    # T_lower_from_aurora = T_upper_from_aurora @ T_lower_from_upper  =>  T_lower_from_upper = inv(T_upper_from_aurora) @ T_lower_from_aurora
    T_lower_from_upper_transform = T_upper_from_aurora_transform.inv() @ T_lower_from_aurora_transform

    print("相対的な変換行列の記録が完了しました。")
    return T_lower_from_upper_transform.matrix

def move_robot_to_goal(arm, aurora, T_lower_from_upper):
    """ロボットをゴール位置に移動する関数"""
    print("ゴール位置を計算中...")
    
    # 現在のupper_probeのAurora座標を取得
    probes = generateProbe(aurora.get_frame())
    upper_probe = probes[1]

    # 現在のupper_probeの位置と姿勢から、ワールド座標系での変換行列を計算
    t_upper_from_aurora = np.array([upper_probe.pos.x, upper_probe.pos.y, upper_probe.pos.z])
    quat_upper_from_aurora = np.array([upper_probe.quat.x, upper_probe.quat.y, upper_probe.quat.z, upper_probe.quat.w])
    R_upper_from_aurora = R.from_quat(quat_upper_from_aurora).as_matrix()
    T_upper_from_aurora_transform = Transform(R_upper_from_aurora, t_upper_from_aurora)

    # T_lower_from_upperをTransformオブジェクトに変換
    T_lower_from_upper_transform = Transform.from_matrix(T_lower_from_upper)

    # lower_probeのゴールとなる変換行列を計算
    # T_lower_from_aurora = T_upper_from_aurora @ T_lower_from_upper
    T_lower_from_aurora_transform_goal = T_upper_from_aurora_transform @ T_lower_from_upper_transform

    # ゴール変換行列から位置とクォータニオンを抽出
    t_lower_from_aurora_goal = T_lower_from_aurora_transform_goal.t
    R_lower_from_aurora_goal = T_lower_from_aurora_transform_goal.R
    quat_lower_from_aurora_goal = R.from_matrix(R_lower_from_aurora_goal).as_quat()

    # main.pyの関数を使用して、ロボットアームの目標姿勢を計算
    T_arm_from_robot = run_pose_transoformation(
        goal_aurora_point=t_lower_from_aurora_goal,
        goal_aurora_quaternion=quat_lower_from_aurora_goal,
        world_calib_csv="robot&aurora/current_code/new_transform/data/aurora_robot_pose_log_202510281545.csv",
        hand_eye_calib_csv="robot&aurora/current_code/new_transform/data/aurora_robot_orientation_log_2020510281902.csv"
    )

    T_arm_from_robot_transform = Transform.from_matrix(T_arm_from_robot)
    t_arm_from_robot = T_arm_from_robot_transform.t
    R_arm_from_robot = T_arm_from_robot_transform.R
    arm_rotvec_from_robot = R.from_matrix(R_arm_from_robot).as_rotvec(degrees=True)

    print(f"ゴール位置 (Robot座標): {t_arm_from_robot}")

    # ロボットアームを段階的に移動
    print("ロボットアームをy,z方向に移動中...")
    arm.set_position(y=t_arm_from_robot[1], z=t_arm_from_robot[2], wait=True, speed=50)

    print("1秒後にx軸方向の移動を開始します...")
    time.sleep(1)
    
    print("ロボットアームをx方向に移動中...")
    arm.set_position(x=t_arm_from_robot[0], wait=True, speed=50)

    angle_pose = [t_arm_from_robot[0], t_arm_from_robot[1], t_arm_from_robot[2], arm_rotvec_from_robot[0], arm_rotvec_from_robot[1], arm_rotvec_from_robot[2]]
    print("ロボットアームの姿勢を調整中...")
    arm.set_position_aa(axis_angle_pose=angle_pose, wait=True, speed=20)

    print("ロボットアームの移動が完了しました。")

    print("目標とした位置・姿勢と現在の位置・姿勢の差分を計算中...")
    # 目標としたT_lower_from_auroraを取得
    T_lower_from_aurora_goal = T_lower_from_aurora_transform_goal.matrix

    # 現在（移動後）のT_lower_from_auroraを取得
    probes_after = generateProbe(aurora.get_frame())
    lower_probe_after = probes_after[0]
    t_lower_from_aurora_after = np.array([lower_probe_after.pos.x, lower_probe_after.pos.y, lower_probe_after.pos.z])
    quat_lower_from_aurora_after = np.array([lower_probe_after.quat.x, lower_probe_after.quat.y, lower_probe_after.quat.z, lower_probe_after.quat.w])
    R_lower_from_aurora_after = R.from_quat(quat_lower_from_aurora_after).as_matrix()
    T_lower_from_aurora_after = Transform(R_lower_from_aurora_after, t_lower_from_aurora_after).matrix

    # 差分を計算
    compute_transform_difference(T_lower_from_aurora_goal, T_lower_from_aurora_after)


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