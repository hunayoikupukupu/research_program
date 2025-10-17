import adaptive_transform  # パッケージ全体をimport

sensor_point_from_robot, sensor_R_vector_from_robot, sensor_quat_from_robot, arm_R_vector_from_robot, arm_quat_from_robot = adaptive_transform.main(
    x_range=(50, 200),                    # X coordinate range (start_value, end_value)
    y_range=(-75, 75),                     # Y coordinate range (start_value, end_value)
    z_range=(-350, -200),                     # Z coordinate range (start_value, end_value)
    divisions=1,                           # Number of divisions (divisions per axis for divisions^3 regions)
    data_file='robot&aurora/current_code/calibration_data/pose_R10--40-60_T20-10--150_SEN-R-40-170-90_SEN-T0-0-0_n0_qn0.csv',  # Data file path
    input_point=[-80.28334,49.89939,-184.9044],             # Coordinates of point to transform [x, y, z]
    input_quaternion=[0.75583,-0.35385,-0.26267,-0.48428]          # Quaternion representing pose [x, y, z, w]
)