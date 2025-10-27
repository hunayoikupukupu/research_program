# 最終的なarm_from_robotを求める
# T_sensor_from_robot = T_aurora_from_robot @ T_sensor_from_aurora
# T_arm_from_robot = T_sensor_from_robot @ T_arm_from_sensor

from scipy.spatial.transform import Rotation as R
from calibration.transformation_utils import Transform

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
    T_sensor_from_aurora_transform = Transform(R_sensor_from_aurora, t_sensor_from_aurora).matrix

    # T_sensor_from_robotを求める
    T_aurora_from_robot_transform = Transform.from_matrix(T_aurora_from_robot)
    T_sensor_from_robot_transform = T_aurora_from_robot_transform @ T_sensor_from_aurora_transform

    # T_arm_from_robotを求める
    T_arm_from_sensor_transform = Transform.from_matrix(T_arm_from_sensor)
    T_arm_from_robot_transform = T_sensor_from_robot_transform @ T_arm_from_sensor_transform

    return T_arm_from_robot_transform.matrix