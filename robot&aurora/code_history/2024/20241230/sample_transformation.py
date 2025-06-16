import numpy as np
import matplotlib.pyplot as plt
import time

from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.probe import generate_probe
from utils.robot_pos import generate_robot

# ロボットとプローブのデータリストを初期化
robot_positions = []  # ロボットの位置リスト
probe_positions = []  # プローブの位置リスト (位置 + quality)

# ロボットとプローブの初期位置
robot_frame = (1, [0.0, 1.0, 2.0, 45.0, 30.0, 60.0])
aurora_frame = (
    [1, 2, 3],  # ポート番号
    [0.1, 0.2, 0.3],  # タイムスタンプ
    [10.0, 20.0, 30.0],  # フレーム番号
    [  # トラッキングデータ (リスト内の各要素はクォータニオン + ベクトル)
        [[0.707, 0.0, 0.707, 0.0, 1.0, 2.0, 3.0]],  # ポート1のデータ
        [[1.0, 0.0, 0.0, 0.0, 4.0, 5.0, 6.0]],      # ポート2のデータ
        [[0.0, 0.707, 0.0, 0.707, 7.0, 8.0, 9.0]]   # ポート3のデータ
    ],
    [0.95, 0.90, 0.85]  # 品質データ
)

# ロボットとプローブの生成
robot = generate_robot(robot_frame)
probes = generate_probe(aurora_frame)

# ロボットの位置を保存
robot_positions.append((robot.pos.x, robot.pos.y, robot.pos.z))

# プローブの位置と品質を保存
for probe in probes:
    probe_positions.append((probe.pos.x, probe.pos.y, probe.pos.z, probe.quality))

# 位置情報を出力
print("Robot Positions:")
for x, y, z in robot_positions:
    print(f"x: {x}, y: {y}, z: {z}")

print("\nProbe Positions:")
for i, (x, y, z, quality) in enumerate(probe_positions):
    print(f"Probe {i+1} - x: {x}, y: {y}, z: {z}, quality: {quality}")