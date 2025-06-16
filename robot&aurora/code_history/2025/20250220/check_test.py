import numpy as np
import time

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.probe import generate_probe
from utils.robot_pos import generate_robot
from utils.transformation_matrix import findTransformation
from utils.coordinate3 import transform_point_and_orientation
from utils.coordinate3 import create_coordinate_system

from scipy.spatial.transform import Rotation as R

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

# Auroraの座標情報と姿勢情報を用意
pos_A = np.array([aurora_pos_x, aurora_pos_y, aurora_pos_z])
print(pos_A)
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

# ロボットの座標情報と姿勢情報を用意
pos_R = np.array([robot_pos_x, robot_pos_y, robot_pos_z])
rotation_R_angles = [robot_rot_x, robot_rot_y, robot_rot_z]
quaternion_R = R.from_euler('xyz', rotation_R_angles, degrees=True).as_quat()


def format_vector(vector):
   """ベクトルを整形された文字列に変換"""
   return "[" + ", ".join([f"{x:.4f}" for x in vector]) + "]"
# 結果の表示
print("\n=== 変換前 ===")
print("Aurora座標系における点Pの座標:", format_vector(pos_A))
print("Aurora座標系における点Pの姿勢 (roll, pitch, yaw):", format_vector(rotation_A_angles))
print("Aurora座標系における点Pの姿勢 (クォータニオン):", format_vector(quaternion_A))
time.sleep(1)
print("\n=== 変換後 ===")
print("ロボット座標系における点Pの座標:", format_vector(pos_R))
print("ロボット座標系における点Pの姿勢 (roll, pitch, yaw):", format_vector(rotation_R_angles))
print("ロボット座標系における点Pの姿勢 (クォータニオン):", format_vector(quaternion_R))
