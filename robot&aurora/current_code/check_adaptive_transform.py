import adaptive_transform  # パッケージ全体をimport

sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = adaptive_transform.main(
    x_range=(75, 175),                    # X coordinate range (start_value, end_value)
    y_range=(-50, 50),                     # Y coordinate range (start_value, end_value)
    z_range=(-350, -250),                     # Z coordinate range (start_value, end_value)
    divisions=1,                           # Number of divisions (divisions per axis for divisions^3 regions)
    data_file='robot&aurora/current_code/calibration_data/pose_R30-120--45_T0-0-0_ARM-64.52-0-168.04_SEN-60--80-150_n0_qn0.csv',  # Data file path
    input_point=[298.15202,-10.57259,204.00635],             # Coordinates of point to transform [x, y, z]
    input_quaternion=[-0.26124,0.87596,0.33182,-0.23313]          # Quaternion representing pose [x, y, z, w]
)