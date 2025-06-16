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

# (140<x<340), (-100<y<100), (25<z<225)
arm.set_position(x=340, y=100, z=225, roll=180, pitch=0, yaw=0, wait=True)
for i in range(10):# xを動かすfor文
    arm.set_position(x=140 + 20*i, y=-100, z=25, wait=True)

for j in range(10):# yを動かすfor文
    arm.set_position(x=340, y=-100 + 20*j, z=25, wait=True)

for i in range(10):# xを動かすfor文
    arm.set_position(x=340 - 20*i, y=100, z=25, wait=True)

for j in range(10):# yを動かすfor文
    arm.set_position(x=140, y=100 - 20*j, z=25, wait=True)

for i in range(10):# xを動かすfor文
    arm.set_position(x=140 + 20*i, y=-100, z=225, wait=True)

for j in range(10):# yを動かすfor文
    arm.set_position(x=340, y=-100 + 20*j, z=225, wait=True)

for i in range(10):# xを動かすfor文
    arm.set_position(x=340 - 20*i, y=100, z=225, wait=True)

for j in range(10):# yを動かすfor文
    arm.set_position(x=140, y=100 - 20*j, z=225, wait=True)

aurora.close()
arm.disconnect()