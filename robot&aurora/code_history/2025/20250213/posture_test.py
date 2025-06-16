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

targetData=np.loadtxt('robot&aurora/20250213/transform_data.csv',skiprows=1,delimiter=',')
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
R_matrix, t = findTransformation(P2, P1)
t_vector = np.array(t)

print("並進ベクトル:\n", np.round(t, 4))
print("回転行列:\n", np.round(R_matrix, 4))

# aurora座標系上の点P(座標情報、姿勢情報)
pos_P_A = np.array([aurora_pos_x, aurora_pos_y, aurora_pos_z])
quaternion_P_A = np.array([aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w])
euler_P_A = R.from_quat(quaternion_P_A).as_euler('xyz', degrees=True)

# 点Pの座標と姿勢を座標系Bへ変換
pos_P_R, orientation_P_R_euler, orientation_P_R_quat = transform_point_and_orientation(pos_P_A, quaternion_P_A, t, R_matrix)

def format_vector(vector):
   """ベクトルを整形された文字列に変換"""
   return "[" + ", ".join([f"{x:.4f}" for x in vector]) + "]"
# 結果の表示
print("\n=== 変換前 ===")
print("Aurora座標系における点Pの座標:", format_vector(pos_P_A))
print("Aurora座標系における点Pの姿勢 (roll, pitch, yaw):", format_vector(euler_P_A))
print("Aurora座標系における点Pの姿勢 (クォータニオン):", format_vector(quaternion_P_A))
time.sleep(1)
print("\n=== 変換後 ===")
print("ロボット座標系における点Pの座標:", format_vector(pos_P_R))
print("ロボット座標系における点Pの姿勢 (roll, pitch, yaw):", format_vector(orientation_P_R_euler))
print("ロボット座標系における点Pの姿勢 (クォータニオン):", format_vector(orientation_P_R_quat))
