"""
Main module for coordinate transformation system.
"""

from .utils import (
    build_coordinate_transformation_model,
    transform_pose,
    print_transformation_results
)


def main(x_range, y_range, z_range, divisions, data_file, input_point=None, input_quaternion=None):
    """
    Execute main processing.
    
    Args:
        x_range (tuple): X-axis range (start_value, end_value)
        y_range (tuple): Y-axis range (start_value, end_value)
        z_range (tuple): Z-axis range (start_value, end_value)
        divisions (int): Number of divisions (divisions per axis for divisions^3 regions)
        data_file (str): Data file path
        input_point (list, optional): Coordinates to transform [x, y, z] (default: None)
        input_quaternion (list, optional): Pose quaternion to transform [x, y, z, w] (default: None)
        
    Returns:
        tuple or None: Transformation result tuple (transformed coordinates, transformed Euler angles, transformed quaternion) or None
    """
    # Build transformation model
    R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer = build_coordinate_transformation_model(
        x_range, y_range, z_range, divisions, data_file
    )
    
    if R_aurora_to_robot_matrices is None:
        return None
    
    # Create method to convert sensor Aurora pose to robot pose
    transformer.create_sensor_quat_from_robot(R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer)

    # Calculate transformation matrices from arm coordinate system to sensor coordinate system
    R_sensor_to_arm_matrices = transformer.calculate_arm_transformations()

    # Display transformation results
    print_transformation_results(R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices)

    # Execute transformation if input is provided
    if input_point is not None and input_quaternion is not None:
        return transform_pose(input_point, input_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices, transformer)
    else:
        # Use default example if no input is provided
        example_point = [250, 0, 100]  # Coordinates of point to transform
        example_quaternion = [0, 0, 0, 1]  # Quaternion representing pose [x, y, z, w]

        transform_pose(example_point, example_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices, transformer)
        
        # Don't return example transformation results (None)
        return None    


if __name__ == "__main__":
    # Set all parameters in one place here (only need to change this)
    result = main(
        x_range=(250, 350),                    # X coordinate range (start_value, end_value)
        y_range=(-50, 50),                     # Y coordinate range (start_value, end_value)
        z_range=(75, 175),                     # Z coordinate range (start_value, end_value)
        divisions=2,                           # Number of divisions (divisions per axis for divisions^3 regions)
        data_file='robot&aurora/current_code/calibration_data/pose_R0-0-45_T100-0-0_ARM180-0-0_SEN0-0-45_n0_qn0.csv',  # Data file path
        input_point=[70.71068,-141.42136,95.0],             # Coordinates of point to transform [x, y, z]
        input_quaternion=[0.48296,-0.22414,0.12941,0.83652]          # Quaternion representing pose [x, y, z, w]
    )