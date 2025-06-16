import numpy as np
from scipy.spatial.transform import Rotation

# オイラー角 (180, 0, 0) に対応するクォータニオンを計算
test_quat = Rotation.from_euler('xyz', [180, 0, 0], degrees=True).as_quat()

print(f"Test Quat: {test_quat}")
# クォータニオンをオイラー角に変換
robot_roll, robot_pitch, robot_yaw = Rotation.from_quat(test_quat).as_euler('xyz', degrees=True)

# 結果を出力
print(f"Roll: {robot_roll}, Pitch: {robot_pitch}, Yaw: {robot_yaw}")

# return 文は関数内でのみ使用可能
# return robot_roll, robot_pitch, robot_yaw