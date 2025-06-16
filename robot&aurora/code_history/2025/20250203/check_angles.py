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

for i in range(2):# xを動かすfor文
    for j in range(2):# xを動かすfor文
        for k in range(2):# xを動かすfor文
            arm.set_position(x=250 + 100*i, y=-100 + 100*j, z=75 + 100*k, wait=True)

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
            euler_A = R.from_quat(quaternion_A).as_euler('xyz', degrees=True)

            # robot座標系の情報
            robot = generate_robot(arm.get_position())
            robot_pos_x = robot.pos.x
            robot_pos_y = robot.pos.y
            robot_pos_z = robot.pos.z
            robot_rot_x = robot.rot.roll
            robot_rot_y = robot.rot.pitch
            robot_rot_z = robot.rot.yaw

            pos_R = np.array([robot_pos_x, robot_pos_y, robot_pos_z])  # Bの原点
            euler_R = [robot_rot_x, robot_rot_y, robot_rot_z]  # x, y, z軸回転角度（度）
            quaternion_R = R.from_euler('xyz', euler_R, degrees=True).as_quat()

            print("Aurora座標系における座標:", np.round(pos_A, 4).tolist())
            print("Aurora座標系における姿勢 (roll, pitch, yaw):", np.round(euler_A, 4).tolist())

aurora.stop_tracking()
aurora.close()