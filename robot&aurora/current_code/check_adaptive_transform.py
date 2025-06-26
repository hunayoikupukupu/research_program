import adaptive_transform  # パッケージ全体をimport

sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = adaptive_transform.main(
    x_range=(250, 350),                    # X coordinate range (start_value, end_value)
    y_range=(-50, 50),                     # Y coordinate range (start_value, end_value)
    z_range=(75, 175),                     # Z coordinate range (start_value, end_value)
    divisions=2,                           # Number of divisions (divisions per axis for divisions^3 regions)
    data_file='robot&aurora/current_code/calibration_data/aurora_robot_pose_log_yaw-.csv',  # Data file path
    input_point=[1.59,5.63,-131.69],             # Coordinates of point to transform [x, y, z]
    input_quaternion=[-0.821,0.571,-0.002,0.011]          # Quaternion representing pose [x, y, z, w]
)