import numpy as np

import csv

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

targetData=np.loadtxt('robot&aurora/20240718/data_pitch0.csv',skiprows=1,delimiter=',')
# 全てのロボット座標とaurora座標
all_robot_x=[]
all_robot_y=[]
all_robot_z=[]
all_aurora_x=[]
all_aurora_y=[]
all_aurora_z=[]
# 変換に使うロボット座標とaurora座標
transform_robot_x = []
transform_robot_y = []
transform_robot_z = []
transform_aurora_x = []
transform_aurora_y = []
transform_aurora_z = []

#csvファイルから要素を抽出
for xyz in targetData:
    all_robot_x.append(float(xyz[0]))
    all_robot_y.append(float(xyz[1]))
    all_robot_z.append(float(xyz[2]))
    all_aurora_x.append(float(xyz[3]))
    all_aurora_y.append(float(xyz[4]))
    all_aurora_z.append(float(xyz[5]))

    # 条件に従って要素を追加
    if float(xyz[0]) < 250 and float(xyz[1]) > -25 and float(xyz[2]) > 105 and float(xyz[1]) < 25 and float(xyz[2]) < 155:
        transform_robot_x.append(float(xyz[0]))
        transform_robot_y.append(float(xyz[1]))
        transform_robot_z.append(float(xyz[2]))
        transform_aurora_x.append(float(xyz[3]))
        transform_aurora_y.append(float(xyz[4]))
        transform_aurora_z.append(float(xyz[5]))

P1 = np.column_stack((transform_robot_x, transform_robot_y, transform_robot_z))
P2 = np.column_stack((transform_aurora_x, transform_aurora_y, transform_aurora_z))
P3 = np.column_stack((all_robot_x, all_robot_y, all_robot_z))
P4 = np.column_stack((all_aurora_x, all_aurora_y, all_aurora_z))

# 変換行列を求める
R2, t2 = findTransformation(P2, P1)

print("Estimated Rotation Matrix:\n", np.round(R2, 4))
print("Estimated Translation Vector:\n", np.round(t2, 4))

# 計算した回転行列R2と並進行列t2を使って返還後の座標を計算
P_translation = np.array(P3) - np.array(t2)
P4_after = np.dot(P_translation, R2)

print("caluculated\n", np.round(P4_after, 4))

#csv出力用の2次元リスト
data = []

N=10

for i in range(N*N*N):
    data.append([np.round(P3[i, 0], 3), np.round(P3[i, 1], 3), np.round(P3[i, 2], 3), np.round(P4[i, 0], 3), np.round(P4[i, 1], 3), np.round(P4[i, 2], 3), np.round(P4_after[i, 0], 3), np.round(P4_after[i, 1], 3), np.round(P4_after[i, 2], 3)])

#CSVファイルに出力
with open("robot&aurora/20240718/p0_centor_back.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)