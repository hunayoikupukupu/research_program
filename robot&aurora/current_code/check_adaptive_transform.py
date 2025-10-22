import adaptive_transform  # パッケージ全体をimport

sensor_point_from_robot, sensor_R_vector_from_robot, sensor_quat_from_robot, arm_R_vector_from_robot, arm_quat_from_robot = adaptive_transform.main(
    x_range=(50, 200),                    # X coordinate range (start_value, end_value)
    y_range=(-75, 75),                     # Y coordinate range (start_value, end_value)
    z_range=(-350, -200),                     # Z coordinate range (start_value, end_value)
    divisions=1,                           # Number of divisions (divisions per axis for divisions^3 regions)
    data_file='robot&aurora/current_code/calibration_data/aurora_robot_pose_log_202510.csv',  # Data file path
    input_point=[50.47,-80.19,-228.23],             # Coordinates of point to transform [x, y, z]
    input_quaternion=[-0.022,-0.039,0.397,0.917]          # Quaternion representing pose [x, y, z, w]
)