import numpy as np
import csv
import time

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.probe import generate_probe
from utils.robot_pos import generate_robot
from utils.transformation_matrix import findTransformation
from utils.posture import findQuatanion, findQuatanion_2

# https://github.com/SciKit-Surgery/scikit-surgerynditracker/tree/master
from sksurgerynditracker.nditracker import NDITracker

# https://github.com/xArm-Developer/xArm-Python-SDK
from xarm.wrapper import XArmAPI

targetData=np.loadtxt('robot&aurora/20250108/transform_data_5.csv',skiprows=1,delimiter=',')
# 全てのロボット座標とaurora座標
all_robot_x=[]
all_robot_y=[]
all_robot_z=[]
all_aurora_x=[]
all_aurora_y=[]
all_aurora_z=[]
# 変換に使うロボット座標とaurora座標
transform_robot_x = [[],[],[],[],[],[],[],[]]
transform_robot_y = [[],[],[],[],[],[],[],[]]
transform_robot_z = [[],[],[],[],[],[],[],[]]
transform_aurora_x = [[],[],[],[],[],[],[],[]]
transform_aurora_y = [[],[],[],[],[],[],[],[]]
transform_aurora_z = [[],[],[],[],[],[],[],[]]

#csv出力用の2次元リスト
data = []

data_count = 0

#csvファイルから要素を抽出
for xyz in targetData:
    if (float(xyz[0])<240):
        part_x = 0
    else:
        part_x = 1
    if (float(xyz[1])<0):
        part_y = 0
    else:
        part_y = 1
    if (float(xyz[2])<125):
        part_z = 0
    else:
        part_z = 1
    
    part_number = int(part_x*1 + part_y*2 + part_z*4)

    transform_robot_x[part_number].append(float(xyz[0]))
    transform_robot_y[part_number].append(float(xyz[1]))
    transform_robot_z[part_number].append(float(xyz[2]))
    transform_aurora_x[part_number].append(float(xyz[3]))
    transform_aurora_y[part_number].append(float(xyz[4]))
    transform_aurora_z[part_number].append(float(xyz[5]))

    if (int(xyz[0]) == 240 or int(xyz[1]) == 0 or int(xyz[2]) == 125):
        if (float(xyz[0])<=240):
            part_x = 0
        else:
            part_x = 1
        if (float(xyz[1])<=0):
            part_y = 0
        else:
            part_y = 1
        if (float(xyz[2])<=125):
            part_z = 0
        else:
            part_z = 1
        
        part_number = int(part_x*1 + part_y*2 + part_z*4)
        transform_robot_x[part_number].append(float(xyz[0]))
        transform_robot_y[part_number].append(float(xyz[1]))
        transform_robot_z[part_number].append(float(xyz[2]))
        transform_aurora_x[part_number].append(float(xyz[3]))
        transform_aurora_y[part_number].append(float(xyz[4]))
        transform_aurora_z[part_number].append(float(xyz[5]))

    data_count += 1

split = 8
R2 = [None] * split
t2 = [None] * split


for part in range(split):
    P1 = np.column_stack((transform_robot_x[part], transform_robot_y[part], transform_robot_z[part]))
    P2 = np.column_stack((transform_aurora_x[part], transform_aurora_y[part], transform_aurora_z[part]))

    # 変換行列を求める
    R2[part], t2[part] = findTransformation(P1, P2)


    # print("Estimated Rotation Matrix:\n", np.round(R2[part], 4))
    # print("Estimated Translation Vector:\n", np.round(t2[part], 4))

# 理想状態の位置を表すaurora座標を設定
aurora_pos_x = -61.01
aurora_pos_y = 0.23
aurora_pos_z = -235.54
P_target = [aurora_pos_x, aurora_pos_y, aurora_pos_z]


# 理想状態の座標にするためのロボットアーム座標を計算
if (float(aurora_pos_x)<240):
    part_x = 0
else:
    part_x = 1
if (float(aurora_pos_y)<0):
    part_y = 0
else:
    part_y = 1
if (float(aurora_pos_z)<125):
    part_z = 0
else:
    part_z = 1
goal_part_number = int(part_x*1 + part_y*2 + part_z*4)


P_translation = np.array(P_target) - np.array(t2[goal_part_number])
P_next = np.dot(P_translation, R2[goal_part_number])

# 理想状態の姿勢を表すauroraクォータニオンを設定
aurora_quat_x = -0.2224
aurora_quat_y = 0.94
aurora_quat_z = -0.1381
aurora_quat_w = 0.2185
aurora_quat = [aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w]

print("aurora_quat:", aurora_quat)

# 理想状態の姿勢にするためのロボットアームオイラー角を計算
robot_roll, robot_pitch, robot_yaw = findQuatanion(R2[goal_part_number], aurora_quat)

euler_angles = [robot_roll, robot_pitch, robot_yaw]

euler_angles_test = findQuatanion_2(R2[goal_part_number], aurora_quat)


print("Estimated Rotation Matrix:\n", np.round(R2[goal_part_number], 4))
print("Estimated Translation Vector:\n", np.round(t2[goal_part_number], 4))
print("P_next:\n", np.round(P_next, 4))
print("Euler_angles:\n", np.round(euler_angles, 4), )
print("Euler_angles_test:\n", np.round(euler_angles_test, 4))

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

#トラッキング開始
aurora.start_tracking()
time.sleep(3)

arm.set_position(x=P_next[0], y=P_next[1], z=P_next[2], roll=robot_roll, pitch=robot_pitch, yaw=robot_yaw, speed=10, wait=True)


probes = generate_probe(aurora.get_frame())
aurora_pos_x_after = probes[0].pos.x
aurora_pos_y_after = probes[0].pos.y
aurora_pos_z_after = probes[0].pos.z
aurora_quat_x_after = probes[0].quat.x
aurora_y_quat_after = probes[0].quat.y
aurora_z_quat_after = probes[0].quat.z
aurora_w_quat_after = probes[0].quat.w
aurora_quality_after = probes[0].quality

# # クォータニオンをオイラー角に変換

# # aurora_pos_xとaurora_pos_x_after、aurora_quat_xとaurora_quat_x_afterを比較

# # サンプルのデータを取得するプログラムを作成して、目標のところまでやりましょう