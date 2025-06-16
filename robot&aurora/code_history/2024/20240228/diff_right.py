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
real_pos_x = -26.23
real_pos_y = -3.32
real_pos_z = -190.37

#トラッキング開始
aurora.start_tracking()
time.sleep(3)

#xyzを変化させながら座標を取得しておく
for i in range(9):# xを動かすfor文
    for j in range(7):# yを動かすfor文
        for k in range(11):# zを動かすfor文
            arm.set_position(x=200+20*i, y=-180+10*j, z=230-20*k, wait=True)
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
with open("robot&aurora/20240228/data_right_0228.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

aurora.close()
arm.disconnect()