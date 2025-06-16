import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.ticker as mticker
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

    # 共分散行列の計算
    H = np.dot(P1_centered.T, P2_centered)

    # 特異値分解を実行
    U, S, V = np.linalg.svd(H, full_matrices=False)
    H2 = np.dot(U, np.dot(np.diag(S), V))


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

targetData=np.loadtxt('robot&aurora/20241008/transform_data.csv',skiprows=1,delimiter=',')
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
#可動範囲(200<x<300, -50<y<50, 80<z<180)
for xyz in targetData:
    all_robot_x.append(float(xyz[0]))
    all_robot_y.append(float(xyz[1]))
    all_robot_z.append(float(xyz[2]))
    all_aurora_x.append(float(xyz[3]))
    all_aurora_y.append(float(xyz[4]))
    all_aurora_z.append(float(xyz[5]))

    if (float(xyz[0])<255):
        part_x = 0
    else:
        part_x = 1
    if (float(xyz[1])<5):
        part_y = 0
    else:
        part_y = 1
    if (float(xyz[2])<135):
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

P1_all = np.column_stack((all_robot_x, all_robot_y, all_robot_z))
P2_all = np.column_stack((all_aurora_x, all_aurora_y, all_aurora_z))

# 変換行列を求める
R2_all, t2_all = findTransformation(P2_all, P1_all)

print("Estimated Rotation Matrix for all:\n", np.round(R2_all, 4))
print("Estimated Translation Vector for all:\n", np.round(t2_all, 4))

# 初期化
transform_matrix = [None] * split  # split の長さを持つリストを作成
transform_vector = [None] * split  # split の長さを持つリストを作成

for part in range(split):

    P1 = np.column_stack((transform_robot_x[part], transform_robot_y[part], transform_robot_z[part]))
    P2 = np.column_stack((transform_aurora_x[part], transform_aurora_y[part], transform_aurora_z[part]))

    # 変換行列を求める
    R2, t2 = findTransformation(P2, P1)

    transform_matrix[part] = R2
    transform_vector[part] = t2

    print("Estimated Rotation Matrix for part", part, ":\n", np.round(transform_matrix[part], 4))
    print("Estimated Translation Vector for part", part, ":\n", np.round(transform_vector[part], 4))

    # 計算した回転行列R2と並進行列t2を使って返還後の座標を計算
    P_translation = np.array(P1) - np.array(t2)
    P2_after = np.dot(P_translation, R2)

    print("caluculated\n", np.round(P2_after, 4))

    N=5

    for l in range(N*N*N):
        data.append([np.round(P1[l, 0], 3), np.round(P1[l, 1], 3), np.round(P1[l, 2], 3), np.round(P2[l, 0], 3), np.round(P2[l, 1], 3), np.round(P2[l, 2], 3), np.round(P2_after[l, 0], 3), np.round(P2_after[l, 1], 3), np.round(P2_after[l, 2], 3)])

#CSVファイルに出力
with open("robot&aurora/20241008/csv_split8.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)