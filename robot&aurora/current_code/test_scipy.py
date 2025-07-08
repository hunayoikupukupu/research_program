from scipy.spatial.transform import Rotation as R

# Roll = -20°, Pitch = 91°, Yaw = 43° （extrinsic zyx）
rA = R.from_euler('zyx', [43, 91, -20], degrees=True)

angles = rA.as_euler('zyx', degrees=True)
quaternion = rA.as_quat()

# 以下出力用
print("Roll: {:.1f}, Pitch: {:.1f}, Yaw: {:.1f}".format(angles[2], angles[1], angles[0]))
print("Quaternion: [{:.3f}, {:.3f}, {:.3f}, {:.3f}]".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

# Roll = 160°, Pitch = 89°, Yaw = -137° （extrinsic zyx）
rA = R.from_euler('zyx', [-137, 89, 160], degrees=True)

angles = rA.as_euler('zyx', degrees=True)
quaternion = rA.as_quat()

# 以下出力用
print("Roll: {:.1f}, Pitch: {:.1f}, Yaw: {:.1f}".format(angles[2], angles[1], angles[0]))
print("Quaternion: [{:.3f}, {:.3f}, {:.3f}, {:.3f}]".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))