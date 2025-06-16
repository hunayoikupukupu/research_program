import numpy as np
import time

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.probe import generate_probe
from utils.robot_pos import generate_robot
from utils.transformation_matrix import findTransformation
from utils.coordinate3 import create_coordinate_system
from utils.coordinate3 import transform_point_and_orientation

from scipy.spatial.transform import Rotation as R

# テストコード

# ここに具体的なAuroraの座標とクォータニオンを入れる
point_P_A = np.array([-11.95, -28.52, -137.05])
quaternion_P = [-0.8208, 0.5685, -0.0299,  0.0463]
euler_P = R.from_quat(quaternion_P).as_euler('xyz', degrees=True)
_, quaternion_P_A, _ = create_coordinate_system(point_P_A, euler_P)

# ④ 座標変換の実行
targetData=np.loadtxt('robot&aurora/20250213/transform_data.csv',skiprows=1,delimiter=',')
# 全てのロボット座標とaurora座標
all_robot_x=[]
all_robot_y=[]
all_robot_z=[]
all_aurora_x=[]
all_aurora_y=[]
all_aurora_z=[]

data_count = 0

#csvファイルから要素を抽出
#可動範囲(250<x<350, -50<y<50, 75<z<175)
for xyz in targetData:
    all_robot_x.append(float(xyz[0]))
    all_robot_y.append(float(xyz[1]))
    all_robot_z.append(float(xyz[2]))
    all_aurora_x.append(float(xyz[3]))
    all_aurora_y.append(float(xyz[4]))
    all_aurora_z.append(float(xyz[5]))

P1 = np.column_stack((all_robot_x, all_robot_y, all_robot_z))
P2 = np.column_stack((all_aurora_x, all_aurora_y, all_aurora_z))

# 並進ベクトルと回転行列を求める
R_matrix, t = findTransformation(P2, P1)

print("並進ベクトル:\n", np.round(t, 4))
print("回転行列:\n", np.round(R_matrix, 4))

# 座標系Bから見た点Pの座標と姿勢を求める
point_P_B, orientation_P_B_euler, orientation_P_B_quat = transform_point_and_orientation(
    point_P_A, quaternion_P_A, t, R_matrix
)

# 結果の表示（小数点第4位で四捨五入）

print("\n=== 変換前 ===")
print("座標系Aにおける点Pの座標:", np.round(point_P_A, 4))
print("座標系Aにおける点Pの姿勢 (roll, pitch, yaw):", np.round(euler_P, 4))
print("座標系Aにおける点Pの姿勢 (クォータニオン):", np.round(quaternion_P_A, 4))

print("\n=== 変換後 ===")
print("座標系Bにおける点Pの座標:", np.round(point_P_B, 4))
print("座標系Bにおける点Pの姿勢 (roll, pitch, yaw):", np.round(orientation_P_B_euler, 4))
print("座標系Bにおける点Pの姿勢 (クォータニオン):", np.round(orientation_P_B_quat, 4))