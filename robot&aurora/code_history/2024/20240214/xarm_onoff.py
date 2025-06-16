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
#センサーの正しい位置
real_pos_x = -23.22
real_pos_y = -11.69
real_pos_z = -232.08

#トラッキング開始
aurora.start_tracking()
time.sleep(3)

for i in range(10):
    #ロボットを起動状態にしてAuroraで座標を取得する
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    time.sleep(2)
    #基準の位置へ移動
    arm.set_position(x=-29.2, y=-333.1, z=57.5, roll=180, pitch=0, yaw=0,wait=True)
    #aurora座標系上でセンサー位置を出力
    probes = generate_probe(aurora.get_frame())
    #print("aurora_pos:" + probes[1].pos.to_str())
    diff = (probes[1].pos.x -real_pos_x)**2 +(probes[1].pos.y -real_pos_y)**2 +(probes[1].pos.z -real_pos_z)**2
    #配列にデータを追加
    data.append(["on", probes[1].pos.x, probes[1].pos.y, probes[1].pos.z, math.sqrt(diff)])

    #ロボットを停止状態にする
    arm.motion_enable(enable=False)
    time.sleep(2)
    #aurora座標系上でセンサー位置を出力
    probes = generate_probe(aurora.get_frame())
    #print("aurora_pos:" + probes[1].pos.to_str())
    diff = (probes[1].pos.x -real_pos_x)**2 +(probes[1].pos.y -real_pos_y)**2 +(probes[1].pos.z -real_pos_z)**2
    #配列にデータを追加
    data.append(["off", probes[1].pos.x, probes[1].pos.y, probes[1].pos.z, math.sqrt(diff)])
    
#CSVファイルに出力
with open("robot&aurora/20240214/onoff6_0214.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

aurora.close()
arm.disconnect()