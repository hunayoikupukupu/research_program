from calibration.world_calibration import WorldCalibration
from calibration.handeye_calibration import HandEyeCalibration
from scipy.spatial.transform import Rotation as R
import numpy as np

def main(goal_point, goal_quaternion, world_calib_csv, hand_eye_calib_csv):

    # ワールドキャリブレーションを実行して T_aurora_from_robot を取得
    world_calib = WorldCalibration(world_calib_csv)
    T_aurora_from_robot = world_calib.run()
    print("T_aurora_from_robot:")
    print(T_aurora_from_robot)
    euler_aurora_from_robot = R.from_matrix(T_aurora_from_robot[:3, :3]).as_euler('zyx', degrees=True)
    t_aurora_from_robot = T_aurora_from_robot[:3, 3]
    print(f"Euler angles (degrees): Roll: {euler_aurora_from_robot[2]:.2f}, Pitch: {euler_aurora_from_robot[1]:.2f}, Yaw: {euler_aurora_from_robot[0]:.2f}")
    print(f"Translation vector: x: {t_aurora_from_robot[0]:.2f}, y: {t_aurora_from_robot[1]:.2f}, z: {t_aurora_from_robot[2]:.2f}")

    # ハンドアイキャリブレーションを実行して T_arm_from_sensor を取得
    hand_eye_calib = HandEyeCalibration(hand_eye_calib_csv, T_aurora_from_robot)
    T_arm_from_sensor = hand_eye_calib.run()
    print("T_arm_from_sensor:")
    print(T_arm_from_sensor)
    euler_arm_from_sensor = R.from_matrix(T_arm_from_sensor[:3, :3]).as_euler('zyx', degrees=True)
    t_arm_from_sensor = T_arm_from_sensor[:3, 3]
    print(f"Euler angles (degrees): Roll: {euler_arm_from_sensor[2]:.2f}, Pitch: {euler_arm_from_sensor[1]:.2f}, Yaw: {euler_arm_from_sensor[0]:.2f}")
    print(f"Translation vector: x: {t_arm_from_sensor[0]:.2f}, y: {t_arm_from_sensor[1]:.2f}, z: {t_arm_from_sensor[2]:.2f}")

    return T_aurora_from_robot


if __name__ == "__main__":
    result = main(
        goal_point=[300, 0, 100],                   # 変換前のsensor_from_aurora [x, y, z]
        goal_quaternion=[0, 0, 0, 1],               # 変換前のsensor_from_aurora [x, y, z, w]
        world_calib_csv="robot&aurora/current_code/new_transform/data/AUfRO_R0-0-0_T0-0-0_ARfSE-R0-0-0_T10-20-30_n0_qn0.csv",   # ワールドキャリブレーション用CSVファイルのパス
        hand_eye_calib_csv="robot&aurora/current_code/new_transform/data/AUfRO_R0-0-0_T0-0-0_ARfSE-R0-0-0_T10-20-30_n0_qn0.csv"  # ハンドアイキャリブレーション用CSVファイルのパス
    )

