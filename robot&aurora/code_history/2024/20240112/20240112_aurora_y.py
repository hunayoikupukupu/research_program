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

#初期位置に動かす
arm.set_position(x=100, y=-380, z=145, wait=True)
time.sleep(1)

#初期位置を出力
aurora.start_tracking()
time.sleep(3)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)


#絶対値で座標祖指定して動かす(y=370)
arm.set_position(x=100, y=-370, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

#絶対値で座標祖指定して動かす(y=360)
arm.set_position(x=100, y=-360, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

#絶対値で座標祖指定して動かす(y=350)
arm.set_position(x=100, y=-350, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

#絶対値で座標祖指定して動かす(y=340)
arm.set_position(x=100, y=-340, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

#絶対値で座標祖指定して動かす(y=330)
arm.set_position(x=100, y=-330, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

#絶対値で座標祖指定して動かす(y=320)
arm.set_position(x=100, y=-320, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

#絶対値で座標祖指定して動かす(y=310)
arm.set_position(x=100, y=-310, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

#絶対値で座標祖指定して動かす(y=300)
arm.set_position(x=100, y=-300, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

#絶対値で座標祖指定して動かす(y=290)
arm.set_position(x=100, y=-290, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

#絶対値で座標祖指定して動かす(y=280)
arm.set_position(x=100, y=-280, z=145, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
time.sleep(1)

aurora.stop_tracking()

aurora.close()
arm.disconnect()