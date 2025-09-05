import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from utils.initialization import initialize_robot

def main():

    
    print("=" * 50)
    print(f"Robotを始動します")
    print("=" * 50)
    print("Ctrl+C で終了")
    print("初期化中...")
    time.sleep(3)
    
    try:
        # ロボットアームを初期化
        arm = initialize_robot()
        
        arm.set_position(x=145, y=0, z=-210, roll=0, pitch=0, yaw=180, wait=True)

        # メインループ
        try:
            base_angle_pose = [145, 0, -210, 0, 0, 180]
            first_angle_pose = [145, 0, -210, 0, 64.51008021, -168.04299911]
            second_angle_pose = [145, 0, -210, 0, -64.51008021, 168.04299911]

            while True:
                arm.set_position_aa(axis_angle_pose=first_angle_pose, speed=50, wait=True)
                print(f"first_angle_pose: {first_angle_pose}")
                # 1秒待機
                time.sleep(1)

                arm.set_position_aa(axis_angle_pose=base_angle_pose, speed=50, wait=True)
                print(f"base_angle_pose: {base_angle_pose}")
                # 1秒待機
                time.sleep(1)

                arm.set_position_aa(axis_angle_pose=second_angle_pose, speed=50, wait=True)
                print(f"second_angle_pose: {second_angle_pose}")
                # 1秒待機
                time.sleep(1)

                arm.set_position_aa(axis_angle_pose=base_angle_pose, speed=50, wait=True)
                print(f"base_angle_pose: {base_angle_pose}")
                # 1秒待機
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nプログラムを終了します...")
    
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()