"""
Utility functions for coordinate transformation system.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from .core import AdaptiveTransform


# =============================================================================
# Coordinate transformation matrix functions (from transformation_matrix.py)
# =============================================================================

def find_transformation(P2, P1):
    """
    座標系2が座標系1に対する変換パラメータを求める
    P2: 座標系2での点群
    P1: 座標系1での点群
    戻り値: 回転行列R, 平行移動ベクトルt (P1 = R * P2 + t)
    """
    # 重心の計算（変数名と内容を一致させる）
    centroid_P1 = np.mean(P1, axis=0)
    centroid_P2 = np.mean(P2, axis=0)

    # 重心を基準に座標をシフト
    P1_centered = P1 - centroid_P1
    P2_centered = P2 - centroid_P2

    # 共分散行列の計算
    H = np.dot(P1_centered.T, P2_centered)

    # 特異値分解を実行
    U, S, V = np.linalg.svd(H, full_matrices=False)

    # 回転行列 R を計算
    R = np.dot(U, V)

    # 右手系の座標系を保つためのチェック
    if np.linalg.det(R) < 0:
        V[-1, :] = -V[-1, :]
        R = np.dot(U, V)  # または np.dot(U, V.T)

    # 並行移動ベクトル t を計算
    t = centroid_P1 - np.dot(R, centroid_P2)

    return R, t


def estimate_transform_matrix(R_arms, R_sensors):
    """
    アーム姿勢とセンサー姿勢から変換行列を推定する
    R_arms: アーム回転行列のリスト
    R_sensors: センサー回転行列のリスト
    戻り値: 変換回転行列
    """
    n = len(R_arms)
    
    # 各観測に対して R_sensor_i * R_arm_i^T を計算
    M_sum = np.zeros((3, 3))
    for i in range(n):
        M_i = np.dot(R_arms[i], R_sensors[i].T)
        M_sum += M_i
    
    # 平均を取る
    M_avg = M_sum / n
    
    # SVD分解で最も近い回転行列を求める
    U, S, Vt = np.linalg.svd(M_avg)
    R_transform = np.dot(U, Vt)
    
    # 右手系を保証（det(R) = 1）
    if np.linalg.det(R_transform) < 0:
        Vt[-1, :] *= -1
        R_transform = np.dot(U, Vt)
    
    return R_transform


def transformation_error(P1, P2, R_matrix, t_vector):
    """
    Calculate transformation error for given transformation parameters.
    
    Args:
        P1 (np.ndarray): Source points (N x 3)
        P2 (np.ndarray): Target points (N x 3)
        R_matrix (np.ndarray): 3x3 rotation matrix
        t_vector (np.ndarray): 3x1 translation vector
        
    Returns:
        tuple: (mean_error, max_error, rmse)
    """
    # Apply transformation
    P1_transformed = (R_matrix @ P1.T).T + t_vector
    
    # Calculate errors
    errors = np.linalg.norm(P2 - P1_transformed, axis=1)
    
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    rmse = np.sqrt(np.mean(errors**2))
    
    return mean_error, max_error, rmse


# =============================================================================
# Model building and transformation functions
# =============================================================================

def build_coordinate_transformation_model(x_range, y_range, z_range, divisions, data_file):
    """
    Build coordinate transformation model.
    
    Args:
        x_range (tuple): X-axis range (start_value, end_value)
        y_range (tuple): Y-axis range (start_value, end_value)
        z_range (tuple): Z-axis range (start_value, end_value)
        divisions (int): Number of divisions (divisions per axis for divisions^3 regions)
        data_file (str): Data file path
        
    Returns:
        tuple: (R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer)
               Rotation matrices, translation vectors, transformation object
    """
    # Expand ranges
    begin_x, end_x = x_range
    begin_y, end_y = y_range
    begin_z, end_z = z_range
    
    # Display parameters
    print(f"Model building parameters:")
    print(f"  Divisions: {divisions} ({divisions}^3 = {divisions**3} regions)")
    print(f"  X-axis range: {begin_x} to {end_x}")
    print(f"  Y-axis range: {begin_y} to {end_y}")
    print(f"  Z-axis range: {begin_z} to {end_z}")
    print(f"  Data file: {data_file}")
    
    # Read CSV file
    try:
        targetData = np.loadtxt(data_file, skiprows=1, delimiter=',')
    except Exception as e:
        print(f"Error: Failed to read data file. {e}")
        return None, None, None
    
    # Initialize AdaptiveTransform class
    transformer = AdaptiveTransform(divisions, begin_x, end_x, begin_y, end_y, begin_z, end_z)
    
    # Process data
    robot_x, robot_y, robot_z, robot_roll, robot_pitch, robot_yaw, aurora_x, aurora_y, aurora_z, aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w = transformer.process_data(targetData)
    
    # Calculate transformation matrices
    R_aurora_to_robot_matrices, T_aurora_to_robot_vectors = transformer.calculate_transformations()
    
    # Display transformation matrix creation results
    successful_regions = sum(1 for matrix in R_aurora_to_robot_matrices if matrix is not None)
    print(f"Rotation matrix and translation vector creation results: {successful_regions}/{len(R_aurora_to_robot_matrices)} regions succeeded")
    
    return R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer


def transform_pose(input_point, input_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices, transformer):
    """
    Transform specified coordinates and pose.
    
    Args:
        input_point (list): Coordinates to transform [x, y, z]
        input_quaternion (list): Pose quaternion to transform [x, y, z, w]
        R_aurora_to_robot_matrices (list): List of rotation matrices
        T_aurora_to_robot_vectors (list): List of translation vectors
        R_sensor_to_arm_matrices (list): List of sensor-to-arm rotation matrices
        transformer (AdaptiveTransform): AdaptiveTransform object
        
    Returns:
        tuple: Transformed coordinates, Euler angles, quaternions
    """
    # Execute coordinate transformation
    sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = transformer.transform_coordinates(
        input_point,
        input_quaternion,
        R_aurora_to_robot_matrices,
        T_aurora_to_robot_vectors,
        R_sensor_to_arm_matrices
    )
    
    # Display transformation results
    print(f"Before transformation (Sensor_from_Aurora): coordinates [{input_point[0]}, {input_point[1]}, {input_point[2]}], quaternion [{input_quaternion[0]}, {input_quaternion[1]}, {input_quaternion[2]}, {input_quaternion[3]}]")

    if sensor_point_from_robot is not None:
        # Round to 4 decimal places
        rounded_sensor_point = np.round(sensor_point_from_robot, 4)
        rounded_sensor_euler = np.round(sensor_euler_from_robot, 4)
        rounded_sensor_quat = np.round(sensor_quat_from_robot, 4)
        print(f"After transformation (Sensor_from_Robot): coordinates [{rounded_sensor_point[0]:.4f}, {rounded_sensor_point[1]:.4f}, {rounded_sensor_point[2]:.4f}], Euler angles [{rounded_sensor_euler[0]:.4f}, {rounded_sensor_euler[1]:.4f}, {rounded_sensor_euler[2]:.4f}], quaternion [{rounded_sensor_quat[0]:.4f}, {rounded_sensor_quat[1]:.4f}, {rounded_sensor_quat[2]:.4f}, {rounded_sensor_quat[3]:.4f}]")
    else:
        print("Transformation failed: No transformation matrix found for specified coordinates")

    if arm_euler_from_robot is not None:
        # Round to 4 decimal places
        rounded_arm_euler = np.round(arm_euler_from_robot, 4)
        rounded_arm_quat = np.round(arm_quat_from_robot, 4)
        print(f"After transformation (Arm_from_Robot): Euler angles [{rounded_arm_euler[0]:.4f}, {rounded_arm_euler[1]:.4f}, {rounded_arm_euler[2]:.4f}], quaternion [{rounded_arm_quat[0]:.4f}, {rounded_arm_quat[1]:.4f}, {rounded_arm_quat[2]:.4f}, {rounded_arm_quat[3]:.4f}]")
    
    return sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot


def print_transformation_results(R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices):
    """
    Display transformation results.
    
    Args:
        R_aurora_to_robot_matrices (list): List of Aurora-to-robot rotation matrices
        T_aurora_to_robot_vectors (list): List of Aurora-to-robot translation vectors
        R_sensor_to_arm_matrices (list): List of sensor-to-arm rotation matrices
    """
    print("R_aurora_to_robot_matrices (first one):")
    if len(R_aurora_to_robot_matrices) > 0 and R_aurora_to_robot_matrices[0] is not None:
        matrix = R_aurora_to_robot_matrices[0]
        for row in matrix:
            print(f"  [{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")
        # Convert to Euler angles (fixed-axis rotation)
        euler_zyx = R.from_matrix(matrix).as_euler('zyx', degrees=True)
        print(f"  Euler angles (zyx, deg): [{euler_zyx[2]:7.2f}, {euler_zyx[1]:7.2f}, {euler_zyx[0]:7.2f}]")
    else:
        print("  Array is empty or None")

    print("\nT_aurora_to_robot_vectors (first one):")
    if len(T_aurora_to_robot_vectors) > 0 and T_aurora_to_robot_vectors[0] is not None:
        vector = T_aurora_to_robot_vectors[0]
        print(f"  [{vector[0]:8.2f}, {vector[1]:8.2f}, {vector[2]:8.2f}]")
    else:
        print("  Array is empty or None")

    print("\nR_sensor_to_arm_matrices (first one):")
    if len(R_sensor_to_arm_matrices) > 0 and R_sensor_to_arm_matrices[0] is not None:
        matrix = R_sensor_to_arm_matrices[0]
        for row in matrix:
            print(f"  [{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")
        # Convert to Euler angles (fixed-axis rotation)
        euler_zyx = R.from_matrix(matrix).as_euler('zyx', degrees=True)
        print(f"  Euler angles (zyx, deg): [{euler_zyx[2]:7.2f}, {euler_zyx[1]:7.2f}, {euler_zyx[0]:7.2f}]")
    else:
        print("  Array is empty or None")


# =============================================================================
# Additional utility functions
# =============================================================================

def transformation_error(P1, P2, R_matrix, t_vector):
    """
    Calculate transformation error for given transformation parameters.
    
    Args:
        P1 (np.ndarray): Source points (N x 3)
        P2 (np.ndarray): Target points (N x 3)
        R_matrix (np.ndarray): 3x3 rotation matrix
        t_vector (np.ndarray): 3x1 translation vector
        
    Returns:
        tuple: (mean_error, max_error, rmse)
    """
    # Apply transformation: P1_predicted = R * P2 + t
    P1_transformed = (R_matrix @ P2.T).T + t_vector
    
    # Calculate errors
    errors = np.linalg.norm(P1 - P1_transformed, axis=1)
    
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    rmse = np.sqrt(np.mean(errors**2))
    
    return mean_error, max_error, rmse


def validate_transformation(P1, P2, R_matrix, t_vector, threshold=1.0):
    """
    Validate transformation quality.
    
    Args:
        P1 (np.ndarray): Target points (N x 3)
        P2 (np.ndarray): Source points (N x 3)
        R_matrix (np.ndarray): 3x3 rotation matrix
        t_vector (np.ndarray): 3x1 translation vector
        threshold (float): Maximum acceptable error
        
    Returns:
        dict: Validation results
    """
    mean_err, max_err, rmse = transformation_error(P1, P2, R_matrix, t_vector)
    
    # Check if rotation matrix is valid
    is_orthogonal = np.allclose(R_matrix @ R_matrix.T, np.eye(3), rtol=1e-6)
    det_is_one = np.allclose(np.linalg.det(R_matrix), 1.0, rtol=1e-6)
    
    validation_results = {
        'mean_error': mean_err,
        'max_error': max_err,
        'rmse': rmse,
        'is_orthogonal': is_orthogonal,
        'determinant_is_one': det_is_one,
        'is_valid_rotation': is_orthogonal and det_is_one,
        'error_within_threshold': mean_err <= threshold,
        'overall_valid': is_orthogonal and det_is_one and mean_err <= threshold
    }
    
    return validation_results

def test_transformation_functions():
    """
    Test the transformation functions with known data.
    """
    print("Testing transformation functions...")
    
    # Create test data
    original_points = np.array([
        [1, 0, 0],
        [0, 1, 0], 
        [0, 0, 1],
        [1, 1, 1]
    ], dtype=float)
    
    # Apply known transformation
    true_R = R.from_euler('zyx', [30, 45, 60], degrees=True).as_matrix()
    true_t = np.array([10, 20, 30])
    
    # P1 = R * P2 + t の関係で変換後の点群を作成
    transformed_points = (true_R @ original_points.T).T + true_t
    
    # Estimate transformation: find_transformation(P2, P1)
    estimated_R, estimated_t = find_transformation(original_points, transformed_points)
    
    # Check results
    R_error = np.linalg.norm(true_R - estimated_R, 'fro')
    t_error = np.linalg.norm(true_t - estimated_t)
    
    print(f"Rotation matrix error: {R_error:.6f}")
    print(f"Translation vector error: {t_error:.6f}")
    
    # Validation
    validation = validate_transformation(transformed_points, original_points, estimated_R, estimated_t)
    print(f"Validation - Mean error: {validation['mean_error']:.6f}")
    print(f"Validation - Is valid rotation: {validation['is_valid_rotation']}")
    print(f"Validation - Overall valid: {validation['overall_valid']}")
    
    return R_error < 1e-10 and t_error < 1e-10