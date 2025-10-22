from calibration.world_calibration import WorldCalibration
import numpy as np

def main(goal_point, goal_quaternion, world_calib_csv):

    # ワールドキャリブレーションを実行して T_robot_from_aurora を取得
    world_calib = WorldCalibration(world_calib_csv)
    T_robot_from_aurora = world_calib.run()
    
    print("T_robot_from_aurora:")
    print(T_robot_from_aurora)
    
    return T_robot_from_aurora


if __name__ == "__main__":
    result = main(
        goal_point=[300, 0, 100],                   # 変換前のsensor_from_aurora [x, y, z]
        goal_quaternion=[0, 0, 0, 1],               # 変換前のsensor_from_aurora [x, y, z, w]
        world_calib_csv="robot&aurora/current_code/new_transform/data/aurora_robot_pose_log_202510.csv"   # ワールドキャリブレーション用CSVファイルのパス
    )

