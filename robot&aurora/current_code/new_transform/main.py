from calibration.world_calibration import WorldCalibration
from calibration.handeye_calibration import HandEyeCalibration
from calibration.transformation_utils import Transform
from scipy.spatial.transform import Rotation as R

def compute_T_arm_from_robot(t_sensor_from_aurora, quaternion_sensor_from_aurora,
                             T_aurora_from_robot,
                             T_arm_from_sensor):
    """
    T_arm_from_robotを計算する関数
    t_sensor_from_aurora: センサーのオイラー角 [x, y, z]
    quaternion_sensor_from_aurora: センサーのクォータニオン [x, y, z, w]
    T_aurora_from_robot: 4x4同次変換行列
    T_arm_from_sensor: 4x4同次変換行列
    戻り値: T_arm_from_robot: 4x4同次変換行列
    """
    # T_sensor_from_auroraを作成
    R_sensor_from_aurora = R.from_quat(quaternion_sensor_from_aurora).as_matrix()
    T_sensor_from_aurora_transform = Transform(R_sensor_from_aurora, t_sensor_from_aurora)

    # T_sensor_from_robotを求める
    T_aurora_from_robot_transform = Transform.from_matrix(T_aurora_from_robot)
    T_sensor_from_robot_transform = T_aurora_from_robot_transform @ T_sensor_from_aurora_transform

    # T_arm_from_robotを求める
    T_arm_from_sensor_transform = Transform.from_matrix(T_arm_from_sensor)
    T_arm_from_robot_transform = T_sensor_from_robot_transform @ T_arm_from_sensor_transform

    return T_arm_from_robot_transform.matrix

def main(goal_aurora_point, goal_aurora_quaternion, world_calib_csv, hand_eye_calib_csv):

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

    # goal_aurora_pointとgoal_aurora_quaternionから目標となるT_arm_from_robotを計算

    T_arm_from_robot = compute_T_arm_from_robot(
        t_sensor_from_aurora=goal_aurora_point,
        quaternion_sensor_from_aurora=goal_aurora_quaternion,
        T_aurora_from_robot=T_aurora_from_robot,
        T_arm_from_sensor=T_arm_from_sensor
    )

    print("Computed T_arm_from_robot:")
    print(T_arm_from_robot)
    euler_arm_from_robot = R.from_matrix(T_arm_from_robot[:3, :3]).as_euler('zyx', degrees=True)
    t_arm_from_robot = T_arm_from_robot[:3, 3]
    print(f"Euler angles (degrees): Roll: {euler_arm_from_robot[2]:.2f}, Pitch: {euler_arm_from_robot[1]:.2f}, Yaw: {euler_arm_from_robot[0]:.2f}")
    rotvec_arm_from_robot = R.from_matrix(T_arm_from_robot[:3, :3]).as_rotvec(degrees=True)
    print(f"Rotation vector (degrees): RX: {rotvec_arm_from_robot[0]:.2f}, RY: {rotvec_arm_from_robot[1]:.2f}, RZ: {rotvec_arm_from_robot[2]:.2f}")
    print(f"Translation vector: x: {t_arm_from_robot[0]:.2f}, y: {t_arm_from_robot[1]:.2f}, z: {t_arm_from_robot[2]:.2f}")

    return T_arm_from_robot


if __name__ == "__main__":

    csv_row = [-120.13629,20.44596,-178.25329,-0.27743,-0.52842,-0.03808,0.80147]
    csv_file_name = "AUFRO_R10--40-60_T20-10--150_ARFSE-R-40-170-90_T0-0-0_n0_qn0"

    result = main(
        goal_aurora_point=[csv_row[0], csv_row[1], csv_row[2]],                   # 変換前のsensor_from_aurora [x, y, z]
        goal_aurora_quaternion=[csv_row[3], csv_row[4], csv_row[5], csv_row[6]],               # 変換前のsensor_from_aurora [x, y, z, w]
        world_calib_csv="robot&aurora/current_code/new_transform/data/" + csv_file_name + ".csv",   # ワールドキャリブレーション用CSVファイルのパス
        hand_eye_calib_csv="robot&aurora/current_code/new_transform/data/" + csv_file_name + ".csv"  # ハンドアイキャリブレーション用CSVファイルのパス
    )

