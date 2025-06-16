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

N = 10
px1_list = []
py1_list = []
pz1_list = []
px2_list = []
py2_list = []
pz2_list = []
for i in range(N):# xを動かすfor文
    for j in range(N):# yを動かすfor文
        for k in range(N):# zを動かすfor文
            arm.set_position(x=300 - 10*i, y=50 - 10*j, z=180 - 10*k, wait=True)
            time.sleep(2)
            robot = generate_robot(arm.get_position())
            probes = generate_probe(aurora.get_frame())
            px1_list.append(robot.pos.x)
            py1_list.append(robot.pos.y)
            pz1_list.append(robot.pos.z)
            px2_list.append(probes[1].pos.x)
            py2_list.append(probes[1].pos.y)
            pz2_list.append(probes[1].pos.z)
            time.sleep(1)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

P1 = np.column_stack((px1_list, py1_list, pz1_list))
P2 = np.column_stack((px2_list, py2_list, pz2_list))

aurora.close()
arm.disconnect()

#csv出力用の2次元リスト
data = []

for i in range(N*N*N):
    data.append([np.round(P1[i, 0], 3), np.round(P1[i, 1], 3), np.round(P1[i, 2], 3), np.round(P2[i, 0], 3), np.round(P2[i, 1], 3), np.round(P2[i, 2], 3)])

#CSVファイルに出力
with open("robot&aurora/20241008/transform_data.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

# plt.legend()
# plt.show()