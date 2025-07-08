import adaptive_transform  # パッケージ全体をimport

sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = adaptive_transform.main(
    x_range=(75, 175),                    # X coordinate range (start_value, end_value)
    y_range=(-50, 50),                     # Y coordinate range (start_value, end_value)
    z_range=(-350, -250),                     # Z coordinate range (start_value, end_value)
    divisions=2,                           # Number of divisions (divisions per axis for divisions^3 regions)
    data_file='robot&aurora/current_code/calibration_data/pose_R20--55-167_T100-25--66_ARM-20-99-43_SEN11--5--130_n0_qn0.csv',  # Data file path
    input_point=[135.1459,185.66957,-90.93071],             # Coordinates of point to transform [x, y, z]
    input_quaternion=[0.46752,0.43368,0.22365,0.73711]          # Quaternion representing pose [x, y, z, w]
)