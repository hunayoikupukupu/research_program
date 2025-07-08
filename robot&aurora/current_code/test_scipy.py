from scipy.spatial.transform import Rotation as R

# Roll = 0°, Pitch = 0°, Yaw = -180° （extrinsic zyx）
r = R.from_euler('zyx', [-180, 0, 0], degrees=True)

# 再度オイラー角に変換
angles = r.as_euler('zyx', degrees=True)

# 以下出力用
print("Roll: {:.1f}, Pitch: {:.1f}, Yaw: {:.1f}".format(angles[2], angles[1], angles[0]))
