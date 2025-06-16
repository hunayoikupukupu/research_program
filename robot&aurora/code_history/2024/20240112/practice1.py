# https://github.com/SciKit-Surgery/scikit-surgerynditracker/tree/master
from sksurgerynditracker.nditracker import NDITracker

# https://github.com/xArm-Developer/xArm-Python-SDK
from xarm.wrapper import XArmAPI

import time
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

#初期位置を出力
aurora.start_tracking()
time.sleep(3)
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())

probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())


#絶対値で座標祖指定して動かす(中心に持ち上げ)
arm.set_position(pitch=-90, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())

aurora.stop_tracking()
arm.stop_lite6_gripper()

aurora.close()
arm.disconnect()