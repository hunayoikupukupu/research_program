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

#csv出力用の2次元リスト
data = []

#差を計算するための変数
diff = 0
real_pos_x = -23.09
real_pos_y = -11.28
real_pos_z = -232.42

#トラッキング開始
aurora.start_tracking()
time.sleep(3)

#初期位置に移動
arm.set_position(x=10, y=-250, z=100, wait=True)
# xを180まで動かすfor文
for i in range(18):
    arm.set_position(x=10+10*i, y=-250, z=100, wait=True)
    time.sleep(2)
    #robot座標系中のセンサー位置を出力
    robot = generate_robot(arm.get_position())
    #print("robot_pos:" + robot.pos.to_str())
    #aurora座標系上でセンサー位置を出力
    probes = generate_probe(aurora.get_frame())
    #print("aurora_pos:" + probes[1].pos.to_str())
    diff = (probes[1].pos.x -real_pos_x)**2 +(probes[1].pos.y -real_pos_y)**2 +(probes[1].pos.z -real_pos_z)**2

    data.append([robot.pos.x, robot.pos.y, robot.pos.z, probes[1].pos.x, probes[1].pos.y, probes[1].pos.z, math.sqrt(diff)])
    time.sleep(1)

# yを-390まで動かすfor文
for i in range(15):
    arm.set_position(x=180, y=-250-10*i, z=100, wait=True)
    time.sleep(2)
    #robot座標系中のセンサー位置を出力
    robot = generate_robot(arm.get_position())
    #print("robot_pos:" + robot.pos.to_str())
    #aurora座標系上でセンサー位置を出力
    probes = generate_probe(aurora.get_frame())
    #print("aurora_pos:" + probes[1].pos.to_str())
    diff = (probes[1].pos.x -real_pos_x)**2 +(probes[1].pos.y -real_pos_y)**2 +(probes[1].pos.z -real_pos_z)**2

    data.append([robot.pos.x, robot.pos.y, robot.pos.z, probes[1].pos.x, probes[1].pos.y, probes[1].pos.z, math.sqrt(diff)])
    time.sleep(1)

# xを10まで動かすfor文
for i in range(18):
    arm.set_position(x=180-10*i, y=-390, z=100, wait=True)
    time.sleep(2)
    #robot座標系中のセンサー位置を出力
    robot = generate_robot(arm.get_position())
    #print("robot_pos:" + robot.pos.to_str())
    #aurora座標系上でセンサー位置を出力
    probes = generate_probe(aurora.get_frame())
    #print("aurora_pos:" + probes[1].pos.to_str())
    diff = (probes[1].pos.x -real_pos_x)**2 +(probes[1].pos.y -real_pos_y)**2 +(probes[1].pos.z -real_pos_z)**2

    data.append([robot.pos.x, robot.pos.y, robot.pos.z, probes[1].pos.x, probes[1].pos.y, probes[1].pos.z, math.sqrt(diff)])
    time.sleep(1)

# yを-250まで動かすfor文
for i in range(15):
    arm.set_position(x=10, y=-390+10*i, z=100, wait=True)
    time.sleep(2)
    #robot座標系中のセンサー位置を出力
    robot = generate_robot(arm.get_position())
    #print("robot_pos:" + robot.pos.to_str())
    #aurora座標系上でセンサー位置を出力
    probes = generate_probe(aurora.get_frame())
    #print("aurora_pos:" + probes[1].pos.to_str())
    diff = (probes[1].pos.x -real_pos_x)**2 +(probes[1].pos.y -real_pos_y)**2 +(probes[1].pos.z -real_pos_z)**2

    data.append([robot.pos.x, robot.pos.y, robot.pos.z, probes[1].pos.x, probes[1].pos.y, probes[1].pos.z, math.sqrt(diff)])
    time.sleep(1)

#CSVファイルに出力
with open("robot&aurora/1lap_0214.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

aurora.close()
arm.disconnect()