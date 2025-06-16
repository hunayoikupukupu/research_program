import numpy as np
from scipy.spatial.transform import Rotation as R

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
from utils.transformation_matrix import findTransformation
from utils.coordinate3 import transform_point_and_orientation
from utils.coordinate3 import create_coordinate_system

# 座標系Aの定義（原点を[0, 0, 0]とする）
origin_A = np.array([0, 0, 0])
euler_A = np.array([0, 0, 0])  # 基準座標系なのでオイラー角も0
origin_A, quat_A, R_A = create_coordinate_system(origin_A, euler_A)

# 座標系Bの定義
# A→Bへの並進ベクトルt1を定義
t1 = np.array([0.0, 0.0, -5.0])

# A→Bへのオイラー角を定義（度数法）
euler_angle = np.array([30, 45, 90])

# 座標系Bを作成
origin_B = t1  # 並進ベクトルが原点となる
origin_B, quat_B, R1 = create_coordinate_system(origin_B, euler_angle)

# 座標系A上の4点を定義
points_A = np.array([
    [6.0, 0.0, 0.0],
    [1.0, 8.0, 12.0],
    [0.0, 1.0, 10.0],
    [0.0, 22.0, 1.0]
])

# 各点を座標系Bに変換
points_B = np.zeros_like(points_A)
for i, point_A in enumerate(points_A):
    # 点の姿勢は考慮しないのでクォータニオンは単位クォータニオンとする
    unit_quat = np.array([0, 0, 0, 1])
    transformed_point, _, _ = transform_point_and_orientation(point_A, unit_quat, t1, R1)
    points_B[i] = transformed_point

# findTransformationを使用してR2とt2を計算
R2, t2 = findTransformation(points_A, points_B)

# 結果の表示
print("定義した変換パラメータ:")
print("t1:", np.round(t1, 4))
print("R1:\n", np.round(R1, 4))
print("\n計算で求めた変換パラメータ:")
print("t2:", np.round(t2, 4))
print("R2:\n", np.round(R2, 4))

# 変換パラメータの比較
t_diff = np.linalg.norm(t1 - t2)
R_diff = np.linalg.norm(R1 - R2)

print("\n変換パラメータの差異:")
print(f"並進ベクトルの差のノルム: {t_diff:.4f}")
print(f"回転行列の差のノルム: {R_diff:.4f}")

# 許容誤差を定義して判定
tolerance = 1e-10
if t_diff < tolerance and R_diff < tolerance:
    print("\n結果: 変換パラメータは一致しています")
else:
    print("\n結果: 変換パラメータに差異があります")

# 変換前後の点の座標を表示
print("\n変換前の点（座標系A）:")
for i, point in enumerate(points_A):
    print(f"点{i+1}: {np.round(point, 4)}")

print("\n変換後の点（座標系B）:")
for i, point in enumerate(points_B):
    print(f"点{i+1}: {np.round(point, 4)}")

# 点Pの座標変換と検証
print("\n点Pの座標変換検証:")

# 座標系Aから見た点Pの位置と姿勢を定義
pos_P_A = np.array([1.0, 3.0, 5.0])
euler_P_A = np.array([0, 45, 0])

# オイラー角をクォータニオンに変換
rotation_P_A = R.from_euler('xyz', euler_P_A, degrees=True)
quat_P_A = rotation_P_A.as_quat()

print("座標系Aから見た点P（元の値）:")
print(f"位置: {np.round(pos_P_A, 4)}")
print(f"オイラー角: {np.round(euler_P_A, 4)}")
print(f"クォータニオン: {np.round(quat_P_A, 4)}")

# A→Bの変換（R2, t2を使用）
pos_P_B, euler_P_B, quat_P_B = transform_point_and_orientation(pos_P_A, quat_P_A, t2, R2)

print("\n座標系Bから見た点P:")
print(f"位置: {np.round(pos_P_B, 4)}")
print(f"オイラー角: {np.round(euler_P_B, 4)}")
print(f"クォータニオン: {np.round(quat_P_B, 4)}")