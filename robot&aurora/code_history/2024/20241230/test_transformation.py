import numpy as np

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.transformation_matrix import findTransformation

targetData=np.loadtxt('robot&aurora/20241230/sample_data.csv',skiprows=1,delimiter=',')
# 全てのロボット座標とaurora座標
all_robot_x=[]
all_robot_y=[]
all_robot_z=[]
all_aurora_x=[]
all_aurora_y=[]
all_aurora_z=[]

data_count = 0

#csvファイルから要素を抽出
for xyz in targetData:
    all_robot_x.append(float(xyz[0]))
    all_robot_y.append(float(xyz[1]))
    all_robot_z.append(float(xyz[2]))
    all_aurora_x.append(float(xyz[3]))
    all_aurora_y.append(float(xyz[4]))
    all_aurora_z.append(float(xyz[5]))

P1 = np.column_stack((all_robot_x, all_robot_y, all_robot_z))
P2 = np.column_stack((all_aurora_x, all_aurora_y, all_aurora_z))

# 変換行列を求める
R, t = findTransformation(P2, P1)

print("Estimated Rotation Matrix for all:\n", np.round(R, 4))
print("Estimated Translation Vector for all:\n", np.round(t, 4))

P_translation = np.array(P1) - np.array(t)
P2_after = np.round(np.dot(P_translation, R), 2)

print("P2_after:\n", P2_after)