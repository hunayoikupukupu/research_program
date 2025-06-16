from utils.pose_formatter import generateRobotArm, generateProbe

# テストデータの準備
frame_data = (
    [0, 1],  # port_numbers
    [1622537812.123, 1622537813.456],  # time_stamps
    [0, 1],  # frame_numbers
    [
        [[0.0, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0], [0.4, 0.5, 0.6, 0.7, 4.0, 5.0, 6.0]]
    ],  # trackings (List of lists containing [w, x, y, z, pos_x, pos_y, pos_z])
    [0.99, 0.95]  # qualities
)

arm_pose_data = (123, [1.0, 2.0, 3.0, 0.1, 0.2, 0.3])  # code and position with rotation

# generateProbeの呼び出し
probes = generateProbe(frame_data)
for probe in probes:
    print(f"Probe Port: {probe.port_number}, Position: ({probe.pos.x}, {probe.pos.y}, {probe.pos.z}), Quality: {probe.quality}")

# generateRobotArmの呼び出し
robot_arm = generateRobotArm(arm_pose_data)
print(f"Robot Arm Code: {robot_arm.code}, Position: ({robot_arm.pos.x}, {robot_arm.pos.y}, {robot_arm.pos.z}), Rotation (Roll, Pitch, Yaw): ({robot_arm.rot.roll}, {robot_arm.rot.pitch}, {robot_arm.rot.yaw})")
