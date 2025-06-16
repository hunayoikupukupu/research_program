import numpy as np
import time
from scipy.spatial.transform import Rotation as R

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
from utils.probe import generate_probe
from utils.robot_pos import generate_robot
from utils.coordinate import transform_point_and_orientation

# https://github.com/SciKit-Surgery/scikit-surgerynditracker/tree/master
from sksurgerynditracker.nditracker import NDITracker

# https://github.com/xArm-Developer/xArm-Python-SDK
from xarm.wrapper import XArmAPI

# ロボットアームで理想状態に移動

#robot初期設定
arm = XArmAPI('192.168.1.155')
arm.connect()
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

#aurora初期設定
aurora = NDITracker(
    {
        "tracker type": "aurora",
        "serial port": "COM3",
        "use quaternions": True,
    }
)
time.sleep(2)
#トラッキング開始
aurora.start_tracking()
time.sleep(2)

# aurora座標系の情報
probes = generate_probe(aurora.get_frame())
aurora_pos_x = probes[0].pos.x
aurora_pos_y = probes[0].pos.y
aurora_pos_z = probes[0].pos.z
aurora_quat_x = probes[0].quat.x
aurora_quat_y = probes[0].quat.y
aurora_quat_z = probes[0].quat.z
aurora_quat_w = probes[0].quat.w

pos_A = np.array([aurora_pos_x, aurora_pos_y, aurora_pos_z])
quaternion_A = np.array([aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w])
rotation_A_angles = R.from_quat(quaternion_A).as_euler('xyz', degrees=True)

# robot座標系の情報
robot = generate_robot(arm.get_position())
robot_pos_x = robot.pos.x
robot_pos_y = robot.pos.y
robot_pos_z = robot.pos.z
robot_rot_x = robot.rot.roll
robot_rot_y = robot.rot.pitch
robot_rot_z = robot.rot.yaw

pos_R = np.array([robot_pos_x, robot_pos_y, robot_pos_z])  # Bの原点
rotation_B_angles = [robot_rot_x, robot_rot_y, robot_rot_z]  # x, y, z軸回転角度（度）
quaternion_B = R.from_euler('xyz', rotation_B_angles, degrees=True).as_quat()
print(pos_R)
print(rotation_B_angles)

# 並進ベクトルの計算

trans_vector_x = pos_R[0] - pos_A[0]
trans_vector_y = pos_R[1] - pos_A[1]
trans_vector_z = pos_R[2] - pos_A[2]
trans_vector = np.array([trans_vector_x, trans_vector_y, trans_vector_z])

# aurora座標系上の点P(座標情報、姿勢情報)
pos_P_A = np.array([aurora_pos_x, aurora_pos_y, aurora_pos_z])
quaternion_P_A = np.array([aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w])
euler_P_A = R.from_quat(quaternion_P_A).as_euler('xyz', degrees=True)

# pos_P_A = [70.93, 14.05, -250.83]
# quaternion_P_A = [-0.3594, 0.9316, -0.0409, 0.0329]
# euler_P_A = R.from_quat(quaternion_P_A).as_euler('xyz', degrees=True)

# 点Pの座標と姿勢を座標系Bへ変換
pos_P_R, orientation_P_R_euler, orientation_P_R_quat = transform_point_and_orientation(pos_P_A, quaternion_P_A, trans_vector, quaternion_B)

print("Aurora座標系における点Pの座標:", np.round(pos_P_A, 4).tolist())
print("Aurora座標系における点Pの姿勢 (roll, pitch, yaw):", np.round(euler_P_A, 4).tolist())
print("Aurora座標系における点Pの姿勢 (クォータニオン):", np.round(quaternion_P_A, 4).tolist())
print("ロボット座標系における点Pの座標:", np.round(pos_P_R, 4).tolist())
print("ロボット座標系における点Pの姿勢 (roll, pitch, yaw):", np.round(orientation_P_R_euler - euler_P_A, 4).tolist())
print("ロボット座標系における点Pの姿勢 (クォータニオン):", np.round(orientation_P_R_quat, 4).tolist())

