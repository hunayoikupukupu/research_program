from calibration.world_calibration import WorldCalibration
from calibration.handeye_calibration import HandEyeCalibration
import numpy as np

def main(goal_point, goal_quaternion, world_calib_csv, hand_eye_calib_csv):

    # ワールドキャリブレーションを実行して T_robot_from_aurora を取得
    world_calib = WorldCalibration(world_calib_csv)
    T_robot_from_aurora = world_calib.run()
    print("T_robot_from_aurora:")
    print(T_robot_from_aurora)

    # ハンドアイキャリブレーションを実行して T_arm_from_sensor を取得
    hand_eye_calib = HandEyeCalibration(hand_eye_calib_csv, T_robot_from_aurora)
    T_arm_from_sensor = hand_eye_calib.run()
    print("T_arm_from_sensor:")
    print(T_arm_from_sensor)

    return T_robot_from_aurora


if __name__ == "__main__":
    result = main(
        goal_point=[300, 0, 100],                   # 変換前のsensor_from_aurora [x, y, z]
        goal_quaternion=[0, 0, 0, 1],               # 変換前のsensor_from_aurora [x, y, z, w]
        world_calib_csv="robot&aurora/current_code/new_transform/data/pose_R50--120-0_T0-10-0_ARM0-80-60_SEN0-20--150_n0_qn0.csv",   # ワールドキャリブレーション用CSVファイルのパス
        hand_eye_calib_csv="robot&aurora/current_code/new_transform/data/pose_R50--120-0_T0-10-0_ARM0-80-60_SEN0-20--150_n0_qn0.csv"  # ハンドアイキャリブレーション用CSVファイルのパス
    )

