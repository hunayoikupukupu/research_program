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

#csv出力用の2次元リスト
data = []

#差を計算するための変数
diff_aurora = 0
diff_robot = 0
#ロボット座標の初期位置を記録
start_robot = generate_robot(arm.get_position())
robot_base_pos_x = start_robot.pos.x
robot_base_pos_y = start_robot.pos.y
robot_base_pos_z = start_robot.pos.z
#Aurora座標の初期位置を記録
start_probes = generate_probe(aurora.get_frame())
aurora_base_pos_x = start_probes[1].pos.x
aurora_base_pos_y = start_probes[1].pos.y
aurora_base_pos_z = start_probes[1].pos.z

#xyzを変化させながら座標を取得しておく
for i in range(5):# xを動かすfor文
    for j in range(5):# yを動かすfor文
        for k in range(5):# zを動かすfor文
            #arm.set_position(pitch=0, wait=True)
            arm.set_position(x=robot_base_pos_x +50 -20*i, y=robot_base_pos_y +50 -20*j, z=robot_base_pos_z +50 -20*k, wait=True)
            #arm.set_position(pitch=-45, wait=True)
            time.sleep(2)
            #robot座標系中のセンサー位置を出力
            robot = generate_robot(arm.get_position())
            #print("robot_pos:" + robot.pos.to_str())
            #aurora座標系上でセンサー位置を出力
            probes = generate_probe(aurora.get_frame())
            #print("aurora_pos:" + probes[1].pos.to_str())
            diff_aurora = (probes[1].pos.x -aurora_base_pos_x)**2 +(probes[1].pos.y -aurora_base_pos_y)**2 +(probes[1].pos.z -aurora_base_pos_z)**2
            diff_robot = (robot.pos.x -robot_base_pos_x)**2 +(robot.pos.y -robot_base_pos_y)**2 +(robot.pos.z -robot_base_pos_z)**2

            data.append([robot.pos.x, robot.pos.y, robot.pos.z, probes[1].pos.x, probes[1].pos.y, probes[1].pos.z, math.sqrt(diff_robot), math.sqrt(diff_aurora), math.sqrt(diff_robot) -math.sqrt(diff_aurora)])
            time.sleep(1)

#CSVファイルに出力
with open("robot&aurora/20240416/data_distance_90degree.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

aurora.close()
arm.disconnect()