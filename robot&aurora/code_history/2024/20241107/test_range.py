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


arm.set_position(x=120, y=-110, z=20, wait=True)

for i in range(10):# xを動かすfor文
    arm.set_position(x=120 + 21*i, y=-110, z=20, wait=True)

for j in range(10):# yを動かすfor文
    arm.set_position(x=330, y=-110 + 22*j, z=20, wait=True)

for i in range(10):# xを動かすfor文
    arm.set_position(x=330 - 21*i, y=110, z=20, wait=True)

for j in range(10):# yを動かすfor文
    arm.set_position(x=120, y=110 - 22*j, z=20, wait=True)

aurora.close()
arm.disconnect()