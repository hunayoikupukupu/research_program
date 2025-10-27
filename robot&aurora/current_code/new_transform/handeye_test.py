from calibration.world_calibration import WorldCalibration
from calibration.handeye_calibration import HandEyeCalibration
from calibration.transformation_utils import Transform
from scipy.spatial.transform import Rotation as R
import numpy as np

# 1つのT_arm_from_robotとT_sensor_from_auroraからT_arm_from_sensorを求める簡易関数
def handeye_calibration_simple(T_arm_from_robot, T_sensor_from_aurora, T_aurora_from_robot):
    """T_arm_from_robot: 4x4同次変換行列
       T_sensor_from_aurora: 4x4同次変換行列
       T_aurora_from_robot: 4x4同次変換行列
       戻り値: T_arm_from_sensor: 4x4同次変換行列
    """
    T_arm_from_robot_transform = Transform.from_matrix(T_arm_from_robot) 
    T_sensor_from_aurora_transform = Transform.from_matrix(T_sensor_from_aurora)
    T_aurora_from_robot_transform = Transform.from_matrix(T_aurora_from_robot)

    # T_sensor_from_robotを求める
    # 計算式：T_sensor_from_robot = T_aurora_from_robot @ T_sensor_from_aurora
    T_sensor_from_robot_transform = T_aurora_from_robot_transform @ T_sensor_from_aurora_transform

    # T_arm_from_sensorを求める
    # 計算式：T_arm_from_robot = T_sensor_from_robot @ T_arm_from_sensor  =>  T_arm_from_sensor = T_sensor_from_robot.inv() @ T_arm_from_robot
    T_arm_from_sensor_transform = T_sensor_from_robot_transform.inv() @ T_arm_from_robot_transform

    return T_arm_from_sensor_transform.matrix

def main(goal_point, goal_quaternion, world_calib_csv, hand_eye_calib_csv, csv_row):

    # ワールドキャリブレーションを実行して T_aurora_from_robot を取得
    world_calib = WorldCalibration(world_calib_csv)
    T_aurora_from_robot = world_calib.run()
    print("T_aurora_from_robot:")
    print(T_aurora_from_robot)
    euler_aurora_from_robot = R.from_matrix(T_aurora_from_robot[:3, :3]).as_euler('zyx', degrees=True)
    t_aurora_from_robot = T_aurora_from_robot[:3, 3]
    print(f"Euler angles (degrees): Roll: {euler_aurora_from_robot[2]:.2f}, Pitch: {euler_aurora_from_robot[1]:.2f}, Yaw: {euler_aurora_from_robot[0]:.2f}")
    print(f"Translation vector: x: {t_aurora_from_robot[0]:.2f}, y: {t_aurora_from_robot[1]:.2f}, z: {t_aurora_from_robot[2]:.2f}")


    # ハンドアイキャリブレーション用のデータを作成
    t_arm_from_robot = [csv_row[0], csv_row[1], csv_row[2]]
    rotvec_arm_from_robot = [csv_row[3], csv_row[4], csv_row[5]]
    R_arm_from_robot = R.from_rotvec(rotvec_arm_from_robot, degrees=True).as_matrix()
    T_arm_from_robot = Transform(R_arm_from_robot, t_arm_from_robot).matrix

    t_sensor_from_aurora = [csv_row[6], csv_row[7], csv_row[8]]
    quaternion_sensor_from_aurora = [csv_row[9], csv_row[10], csv_row[11], csv_row[12]]
    R_sensor_from_aurora = R.from_quat(quaternion_sensor_from_aurora).as_matrix()
    T_sensor_from_aurora = Transform(R_sensor_from_aurora, t_sensor_from_aurora).matrix

    # 簡易的なハンドアイキャリブレーションを実行して T_arm_from_sensor を取得
    T_arm_from_sensor = handeye_calibration_simple(T_arm_from_robot, T_sensor_from_aurora, T_aurora_from_robot)
    print("T_arm_from_sensor:")
    print(T_arm_from_sensor)
    euler_arm_from_sensor = R.from_matrix(T_arm_from_sensor[:3, :3]).as_euler('zyx', degrees=True)
    t_arm_from_sensor = T_arm_from_sensor[:3, 3]
    print(f"Euler angles (degrees): Roll: {euler_arm_from_sensor[2]:.2f}, Pitch: {euler_arm_from_sensor[1]:.2f}, Yaw: {euler_arm_from_sensor[0]:.2f}")
    print(f"Translation vector: x: {t_arm_from_sensor[0]:.2f}, y: {t_arm_from_sensor[1]:.2f}, z: {t_arm_from_sensor[2]:.2f}")

    return T_aurora_from_robot, T_arm_from_sensor


if __name__ == "__main__":
    result = main(
        goal_point=[300, 0, 100],                   # 変換前のsensor_from_aurora [x, y, z]
        goal_quaternion=[0, 0, 0, 1],               # 変換前のsensor_from_aurora [x, y, z, w]
        world_calib_csv="robot&aurora/current_code/new_transform/data/aurora_robot_pose_log_20251024.csv",   # ワールドキャリブレーション用CSVファイルのパス
        hand_eye_calib_csv="robot&aurora/current_code/new_transform/data/aurora_robot_pose_log_20251024.csv",  # ハンドアイキャリブレーション用CSVファイルのパス
        csv_row=[200.0,45.0,-350.0,-0.0,-0.0,180.0,-99.87,34.54,-82.56,0.005,-0.01,0.558,0.83]
    )
