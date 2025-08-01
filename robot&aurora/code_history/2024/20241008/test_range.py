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

N = 2
px1_list = []
py1_list = []
pz1_list = []
px2_list = []
py2_list = []
pz2_list = []
for i in range(N):# xを動かすfor文
    for j in range(N):# yを動かすfor文
        for k in range(N):# zを動かすfor文
            arm.set_position(x=300 - 100*i, y=50 - 100*j, z=180 - 100*k, wait=True)
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

print(px1_list)
print(py1_list)
print(pz1_list)
print(px2_list)
print(py2_list)
print(pz2_list)

aurora.close()
arm.disconnect()