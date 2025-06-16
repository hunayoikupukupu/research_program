import numpy as np
import csv
import time

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.probe import generate_probe
from utils.robot_pos import generate_robot
from utils.transformation_matrix import findTransformation
from utils.posture import findQuatanion, findQuatanion_2

# https://github.com/SciKit-Surgery/scikit-surgerynditracker/tree/master
from sksurgerynditracker.nditracker import NDITracker

# https://github.com/xArm-Developer/xArm-Python-SDK
from xarm.wrapper import XArmAPI

targetData=np.loadtxt('robot&aurora/20250108/transform_data_5.csv',skiprows=1,delimiter=',')
# 全てのロボット座標とaurora座標
all_robot_x=[]
all_robot_y=[]
all_robot_z=[]
all_aurora_x=[]
all_aurora_y=[]
all_aurora_z=[]
# 変換に使うロボット座標とaurora座標
transform_robot_x = [[],[],[],[],[],[],[],[]]
transform_robot_y = [[],[],[],[],[],[],[],[]]
transform_robot_z = [[],[],[],[],[],[],[],[]]
transform_aurora_x = [[],[],[],[],[],[],[],[]]
transform_aurora_y = [[],[],[],[],[],[],[],[]]
transform_aurora_z = [[],[],[],[],[],[],[],[]]

#csv出力用の2次元リスト
data = []

data_count = 0

#csvファイルから要素を抽出
for xyz in targetData:
    if (float(xyz[0])<240):
        part_x = 0
    else:
        part_x = 1
    if (float(xyz[1])<0):
        part_y = 0
    else:
        part_y = 1
    if (float(xyz[2])<125):
        part_z = 0
    else:
        part_z = 1
    
    part_number = int(part_x*1 + part_y*2 + part_z*4)

    transform_robot_x[part_number].append(float(xyz[0]))
    transform_robot_y[part_number].append(float(xyz[1]))
    transform_robot_z[part_number].append(float(xyz[2]))
    transform_aurora_x[part_number].append(float(xyz[3]))
    transform_aurora_y[part_number].append(float(xyz[4]))
    transform_aurora_z[part_number].append(float(xyz[5]))

    if (int(xyz[0]) == 240 or int(xyz[1]) == 0 or int(xyz[2]) == 125):
        if (float(xyz[0])<=240):
            part_x = 0
        else:
            part_x = 1
        if (float(xyz[1])<=0):
            part_y = 0
        else:
            part_y = 1
        if (float(xyz[2])<=125):
            part_z = 0
        else:
            part_z = 1
        
        part_number = int(part_x*1 + part_y*2 + part_z*4)
        transform_robot_x[part_number].append(float(xyz[0]))
        transform_robot_y[part_number].append(float(xyz[1]))
        transform_robot_z[part_number].append(float(xyz[2]))
        transform_aurora_x[part_number].append(float(xyz[3]))
        transform_aurora_y[part_number].append(float(xyz[4]))
        transform_aurora_z[part_number].append(float(xyz[5]))

    data_count += 1

split = 8
R2 = [None] * split
t2 = [None] * split


for part in range(split):
    P1 = np.column_stack((transform_robot_x[part], transform_robot_y[part], transform_robot_z[part]))
    P2 = np.column_stack((transform_aurora_x[part], transform_aurora_y[part], transform_aurora_z[part]))

    # 変換行列を求める
    R2[part], t2[part] = findTransformation(P1, P2)

# ロボットアームで理想状態に移動

#robot初期設定
arm = XArmAPI('192.168.1.155')
arm.connect()
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(3)
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

# 理想状態の位置を表すaurora座標を設定
aurora_pos_x = [-61.01,-86.4,-80.16,-48.73,34.77]
aurora_pos_y = [0.23,58.41,60.82,-79.55,1.04]
aurora_pos_z = [-235.54,-113.33,-260.09,-223.97,-161.07]


aurora_pos_x_after = []
aurora_pos_y_after = []
aurora_pos_z_after = []
aurora_quality_after = []

for i in range (5):
    # 理想状態の座標にするためのロボットアーム座標を計算
    if (float(aurora_pos_x[i])<240):
        part_x = 0
    else:
        part_x = 1
    if (float(aurora_pos_y[i])<0):
        part_y = 0
    else:
        part_y = 1
    if (float(aurora_pos_z[i])<125):
        part_z = 0
    else:
        part_z = 1
    goal_part_number = int(part_x*1 + part_y*2 + part_z*4)

    P_target = [aurora_pos_x[i], aurora_pos_y[i], aurora_pos_z[i]]

    P_translation = np.array(P_target) - np.array(t2[goal_part_number])
    P_next = np.dot(P_translation, R2[goal_part_number])

    arm.set_position(x=P_next[0], y=P_next[1], z=P_next[2], wait=True)
    time.sleep(1)

    probes = generate_probe(aurora.get_frame())
    aurora_pos_x_after.append(probes[0].pos.x)
    aurora_pos_y_after.append(probes[0].pos.y)
    aurora_pos_z_after.append(probes[0].pos.z)
    aurora_quality_after.append(probes[0].quality)

    print("goal_pos:",aurora_pos_x[i],aurora_pos_y[i],aurora_pos_z[i])
    print("real_pos", np.round(aurora_pos_x_after[i], 4), np.round(aurora_pos_y_after[i], 4), np.round(aurora_pos_z_after[i], 4))
    print("diff:", np.round(aurora_pos_x[i]-aurora_pos_x_after[i], 4), np.round(aurora_pos_y[i]-aurora_pos_y_after[i], 4), np.round(aurora_pos_z[i]-aurora_pos_z_after[i], 4))

# # クォータニオンをオイラー角に変換

# # aurora_pos_xとaurora_pos_x_after、aurora_quat_xとaurora_quat_x_afterを比較

# # サンプルのデータを取得するプログラムを作成して、目標のところまでやりましょう