import adaptive_transform  # パッケージ全体をimport

sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = adaptive_transform.main(
    x_range=(100, 150),                    # X coordinate range (start_value, end_value)
    y_range=(-25, 25),                     # Y coordinate range (start_value, end_value)
    z_range=(-325, -275),                     # Z coordinate range (start_value, end_value)
    divisions=1,                           # Number of divisions (divisions per axis for divisions^3 regions)
    data_file='robot&aurora/current_code/calibration_data/aurora_robot_pose_log.csv',  # Data file path
    input_point=[3.15,-30.72,-86.57],             # Coordinates of point to transform [x, y, z]
    input_quaternion=[-0.634,0.774,0.007,0.007]          # Quaternion representing pose [x, y, z, w]
)