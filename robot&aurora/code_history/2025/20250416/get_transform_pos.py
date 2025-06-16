import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# https://github.com/SciKit-Surgery/scikit-surgerynditracker/tree/master
from sksurgerynditracker.nditracker import NDITracker

# https://github.com/xArm-Developer/xArm-Python-SDK
from xarm.wrapper import XArmAPI

import time
import csv
import math

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
    
from utils.probe import generate_probe
from utils.robot_pos import generate_robot

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

N = 5
D = 100 / N
robot_x_list = []
robot_y_list = []
robot_z_list = []
aurora_x_list = []
aurora_y_list = []
aurora_z_list = []
aurora_quality_list = []
for i in range(N+1):# xを動かすfor文
    for j in range(N+1):# yを動かすfor文
        for k in range(N+1):# zを動かすfor文
            arm.set_position(x=350 - D*i, y=50 - D*j, z=175 - D*k, wait=True)
            time.sleep(1)
            robot = generate_robot(arm.get_position())
            probes = generate_probe(aurora.get_frame())
            robot_x_list.append(robot.pos.x)
            robot_y_list.append(robot.pos.y)
            robot_z_list.append(robot.pos.z)
            aurora_x_list.append(probes[0].pos.x)
            aurora_y_list.append(probes[0].pos.y)
            aurora_z_list.append(probes[0].pos.z)
            aurora_quality_list.append(probes[0].quality)
            time.sleep(1)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

P1 = np.column_stack((robot_x_list, robot_y_list, robot_z_list))
P2 = np.column_stack((aurora_x_list, aurora_y_list, aurora_z_list, aurora_quality_list))

aurora.close()
arm.disconnect()

#csv出力用の2次元リスト
data = []

for i in range((N+1)*(N+1)*(N+1)):
    data.append([np.round(P1[i, 0], 3), np.round(P1[i, 1], 3), np.round(P1[i, 2], 3), np.round(P2[i, 0], 3), np.round(P2[i, 1], 3), np.round(P2[i, 2], 3), np.round(P2[i, 3], 3)])

#CSVファイルに出力
with open("robot&aurora/20250416/transform_data.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

# plt.legend()
# plt.show()