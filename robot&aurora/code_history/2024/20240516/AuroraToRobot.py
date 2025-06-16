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

def trans_and_rot_EulerZYX(T, R, P):
    # 座標系2での座標を計算
    rotation_matrix = eulerZYX(np.deg2rad(R[0]), np.deg2rad(R[1]), np.deg2rad(R[2]))  # X, Y, Z
    P_translation = np.array(P) - np.array(T)
    Pt = np.dot(P_translation, rotation_matrix)
    return Pt

def eulerZYX(ax, ay, az):
    # XYZオイラー角を回転行列に変換
    R1 = np.array([[1, 0, 0],
                   [0, np.cos(ax), -np.sin(ax)],
                   [0, np.sin(ax), np.cos(ax)]])

    R2 = np.array([[np.cos(ay), 0, np.sin(ay)],
                   [0, 1, 0],
                   [-np.sin(ay), 0, np.cos(ay)]])

    R3 = np.array([[np.cos(az), -np.sin(az), 0],
                   [np.sin(az), np.cos(az), 0],
                   [0, 0, 1]])
    R = np.dot(R1, np.dot(R2, R3))
    # R = R3*R2*R1
    return R

def findTransformation(P2, P1):
    # 座標系2が座標系1に対してどれだけ回転しているか(R)，どれだけ並行移動しているか(t)を求める

    # 重心の計算
    centroid_P1 = np.mean(P2, axis=0)
    centroid_P2 = np.mean(P1, axis=0)

    # 重心を基準に座標をシフト
    P1_centered = P2 - centroid_P1
    P2_centered = P1 - centroid_P2
    # print("centroid_P1")
    # print(centroid_P1)
    # print("centroid_P2")
    # print(centroid_P2)
    # print("P1_centered")
    # print(P1_centered)
    # print("P2_centered")
    # print(P2_centered)

    # 共分散行列の計算
    H = np.dot(P1_centered.T, P2_centered)
    # print("H")
    # print(H)
    # 特異値分解を実行
    U, S, V = np.linalg.svd(H, full_matrices=False)
    H2 = np.dot(U, np.dot(np.diag(S), V))
    # print("H2")
    # print(H2)
    # print("U")
    # print(U)
    # print("S")
    # print(S)
    # print("V")
    # print(V)


    # 回転行列 R を計算
    R = np.dot(V.T, U.T)

    # 並行移動ベクトル t を計算
    t = centroid_P2 - np.dot(R, centroid_P1)

    # 回転行列が右手系の座標系を保つためのチェック
    if np.linalg.det(R) < 0:
        V[-1, :] = -V[-1, :]
        R = np.dot(V.T, U.T)

    return R, t

# 座標系1の定義
def draw_axis(ax, origin, rot_angles, length, linewidth):
    O = origin
    ag = rot_angles
    R = RotationMatrix(ag[0], ag[1], ag[2])

    vec = np.dot(R, np.array([length, 0, 0]))
    ax.quiver(O[0], O[1], O[2], vec[0], vec[1], vec[2], linewidth=linewidth, color='blue')  # x-axis

    vec = np.dot(R, np.array([0, length, 0]))
    ax.quiver(O[0], O[1], O[2], vec[0], vec[1], vec[2], linewidth=linewidth, color='red')  # y-axis

    vec = np.dot(R, np.array([0, 0, length]))
    ax.quiver(O[0], O[1], O[2], vec[0], vec[1], vec[2], linewidth=linewidth, color='green')  # z-axis

def RotationMatrix(ax, ay, az):
    ax = np.deg2rad(ax)
    ay = np.deg2rad(ay)
    az = np.deg2rad(az)

    R1 = np.array([[1, 0, 0],
                   [0, np.cos(ax), -np.sin(ax)],
                   [0, np.sin(ax), np.cos(ax)]])

    R2 = np.array([[np.cos(ay), 0, np.sin(ay)],
                   [0, 1, 0],
                   [-np.sin(ay), 0, np.cos(ay)]])

    R3 = np.array([[np.cos(az), -np.sin(az), 0],
                   [np.sin(az), np.cos(az), 0],
                   [0, 0, 1]])
    R = R3*R2*R1
    return R

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

N = 5
px1_list = []
py1_list = []
pz1_list = []
px2_list = []
py2_list = []
pz2_list = []
for i in range(N):# xを動かすfor文
    for j in range(N):# yを動かすfor文
        for k in range(N):# zを動かすfor文
            arm.set_position(x=160 + 20*i, y=-50 + 20*j, z=80 + 20*k, wait=True)
            time.sleep(2)
            robot = generate_robot(arm.get_position())
            probes = generate_probe(aurora.get_frame())
            px1_list.append(robot.pos.x)
            py1_list.append(robot.pos.y)
            pz1_list.append(robot.pos.z)
            px2_list.append(probes[1].pos.x)
            py2_list.append(probes[1].pos.y)
            pz2_list.append(probes[1].pos.z)
            time.sleep(1)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

P1 = np.column_stack((px1_list, py1_list, pz1_list))
P2 = np.column_stack((px2_list, py2_list, pz2_list))


# 変換行列を求める
R2, t2 = findTransformation(P2, P1)

print("Estimated Rotation Matrix:\n", np.round(R2, 4))
print("Estimated Translation Vector:\n", np.round(t2, 4))

print("P1")
print(P1)
print("P2")
print(np.round(P2, 4))

# 計算した回転行列R2と並進行列t2を使って返還後の座標を計算
P_translation = np.array(P1) - np.array(t2)
P2_after = np.dot(P_translation, R2)

print("caluculated\n", np.round(P2_after, 4))

aurora.close()
arm.disconnect()

#csv出力用の2次元リスト
data = []

for i in range(N*N*N):
    data.append([np.round(P1[i, 0], 3), np.round(P1[i, 1], 3), np.round(P1[i, 2], 3), np.round(P2[i, 0], 3), np.round(P2[i, 1], 3), np.round(P2[i, 2], 3), np.round(P2_after[i, 0], 3), np.round(P2_after[i, 1], 3), np.round(P2_after[i, 2], 3)])

#CSVファイルに出力
with open("robot&aurora/20240516/data_transformation.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

# plt.legend()
# plt.show()