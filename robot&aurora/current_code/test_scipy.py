from scipy.spatial.transform import Rotation as R

# Roll = 30°, Pitch = 45°, Yaw = 90° （extrinsic zyx）
r = R.from_euler('zyx', [30, 45, 90], degrees=True)

# 再度オイラー角に変換
angles = r.as_euler('zyx', degrees=True)
# angles = [angles[2], angles[1], angles[0]]  # zyx -> xyz

print(angles)  # 期待と異なる符号が返ることがある
