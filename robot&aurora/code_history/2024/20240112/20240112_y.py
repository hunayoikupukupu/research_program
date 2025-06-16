# https://github.com/SciKit-Surgery/scikit-surgerynditracker/tree/master
from sksurgerynditracker.nditracker import NDITracker

# https://github.com/xArm-Developer/xArm-Python-SDK
from xarm.wrapper import XArmAPI

import time

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

#初期位置に動かしてアームを閉じる
arm.set_position(x=65, y=-335, z=175, wait=True)
arm.open_lite6_gripper()
time.sleep(2)
arm.set_position(x=65, y=-335, z=150, wait=True)
time.sleep(1)
arm.close_lite6_gripper()
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


#絶対値で座標祖指定して動かす(中心に持ち上げ)
arm.set_position(x=65, y=-335, z=200, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())

#絶対値で座標祖指定して動かす(左に移動)
arm.set_position(x=65, y=-365, z=200, wait=True)
time.sleep(2)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())

#絶対値で座標祖指定して動かす(右に移動)
arm.set_position(x=65, y=-305, z=200, wait=True)
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