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
from utils.sensorToArm import compute_rotation_matrix_R2
from utils.sensorToArm import verify_r2_calculation

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

# 実際の値を取得

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



# 座標・姿勢変換検証

targetData=np.loadtxt('robot&aurora/20250416/transform_data.csv',skiprows=1,delimiter=',')
# 全てのロボット座標とaurora座標
all_robot_x=[]
all_robot_y=[]
all_robot_z=[]
all_aurora_x=[]
all_aurora_y=[]
all_aurora_z=[]

data_count = 0

#csvファイルから要素を抽出
#可動範囲(250<x<350, -50<y<50, 75<z<175)
for xyz in targetData:
    all_robot_x.append(float(xyz[0]))
    all_robot_y.append(float(xyz[1]))
    all_robot_z.append(float(xyz[2]))
    all_aurora_x.append(float(xyz[3]))
    all_aurora_y.append(float(xyz[4]))
    all_aurora_z.append(float(xyz[5]))

P1 = np.column_stack((all_robot_x, all_robot_y, all_robot_z))
P2 = np.column_stack((all_aurora_x, all_aurora_y, all_aurora_z))

# 並進ベクトルと回転行列を求める
R1, T1 = findTransformation(P2, P1)

print("並進ベクトル:\n", np.round(T1, 4))
print("回転行列:\n", np.round(R1, 4))

# ロボット座標系から見たセンサーの座標と姿勢を求める
pos_S_R, orientation_S_R_euler, orientation_S_R_quat = transform_point_and_orientation(
    pos_A, quaternion_A, T1, R1
)

# 結果の表示（小数点第4位で四捨五入）

print("\n=== 変換前 ===")
print("座標系Aにおけるセンサーの座標:", np.round(pos_A, 4))
print("座標系Aにおけるセンサーの姿勢 (roll, pitch, yaw):", np.round(rotation_A_angles, 4))
print("座標系Aにおけるセンサーの姿勢 (クォータニオン):", np.round(quaternion_A, 4))

print("\n=== 変換後 ===")
print("座標系Rにおけるセンサーの座標:", np.round(pos_S_R, 4))
print("座標系Rにおけるセンサーの姿勢 (roll, pitch, yaw):", np.round(orientation_S_R_euler, 4))
print("座標系Rにおけるセンサーの姿勢 (クォータニオン):", np.round(orientation_S_R_quat, 4))


# R2の計算
R2 = compute_rotation_matrix_R2(quaternion_A, rotation_R_angles, R1)
print("R2 (センサー⇒アーム):")
print(R2)

# 検証: 理想センサー姿勢から理想アーム姿勢を計算
# ideal_sensor_pose_aurora_quat = quaternion_A  # 理想クォータニオン [x, y, z, w]
ideal_sensor_pose_aurora_quat = [-0.2886,  0.9561, -0.0244,  0.0424]

ideal_sensor_position_aurora = [ -28.98,   -4.6,  -164.44]  # 理想センサー位置（Aurora座標系）

ideal_arm_pose_euler, ideal_arm_position = verify_r2_calculation(R2, ideal_sensor_pose_aurora_quat, ideal_sensor_position_aurora, T1, R1)

print("\n理想アーム姿勢 (オイラー角, 度):")
print(ideal_arm_pose_euler)
print("\n理想アーム位置:")
print(ideal_arm_position)