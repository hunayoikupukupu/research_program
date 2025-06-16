# https://github.com/SciKit-Surgery/scikit-surgerynditracker/tree/master
from sksurgerynditracker.nditracker import NDITracker

# https://github.com/xArm-Developer/xArm-Python-SDK
from xarm.wrapper import XArmAPI

import time
import csv


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

#csv出力用の2次元リスト
data = []

#トラッキング開始
aurora.start_tracking()
time.sleep(3)

#初期位置に動かす
arm.set_position(x=100, y=-335, z=335, wait=True)
time.sleep(1)
#robot座標系中のセンサー位置を出力
robot = generate_robot(arm.get_position())
print("robot_pos:" + robot.pos.to_str())
#aurora座標系上でセンサー位置を出力
probes = generate_probe(aurora.get_frame())
print("aurora_pos:" + probes[1].pos.to_str())
data.append([probes[1].pos.x, probes[1].pos.y, probes[1].pos.z])
time.sleep(1)

for i in range(19):
    #相対位置で動かす(z-=10)
    arm.set_position(z=-10, relative=True, wait=True)
    time.sleep(2)
    #robot座標系中のセンサー位置を出力
    robot = generate_robot(arm.get_position())
    print("robot_pos:" + robot.pos.to_str())
    #aurora座標系上でセンサー位置を出力
    probes = generate_probe(aurora.get_frame())
    print("aurora_pos:" + probes[1].pos.to_str())
    data.append([probes[1].pos.x, probes[1].pos.y, probes[1].pos.z])
    time.sleep(1)

aurora.stop_tracking()

#CSVファイルに出力
with open("robot&aurora/z_record2.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

aurora.close()
arm.disconnect()