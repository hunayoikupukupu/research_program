"""
Core module containing the AdaptiveTransform class for coordinate transformation.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

# Note: find_transformation and estimate_transform_matrix are now in utils.py
# They will be imported directly in the calculate_transformations and calculate_arm_transformations methods


class AdaptiveTransform:
    """
    Adaptive coordinate transformation system that divides 3D space into regions
    and maintains separate transformation matrices for each region.
    """
    
    def __init__(self, divisions_per_axis, begin_x, end_x, begin_y, end_y, begin_z, end_z):  
        """
        Initialize the adaptive transformation system.
        
        Args:
            divisions_per_axis (int): Number of divisions per axis (2->8 regions, 3->27 regions, 4->64 regions)
            begin_x, end_x (float): X-axis range
            begin_y, end_y (float): Y-axis range
            begin_z, end_z (float): Z-axis range
        """
        self.divisions = divisions_per_axis
        self.region_count = divisions_per_axis ** 3

        self.begin_x = begin_x
        self.end_x = end_x
        self.begin_y = begin_y
        self.end_y = end_y
        self.begin_z = begin_z
        self.end_z = end_z
        
        # Initialize lists to store data for each region
        self.transform_robot_x = [[] for _ in range(self.region_count)]
        self.transform_robot_y = [[] for _ in range(self.region_count)]
        self.transform_robot_z = [[] for _ in range(self.region_count)]
        self.transform_robot_roll = [[] for _ in range(self.region_count)]
        self.transform_robot_pitch = [[] for _ in range(self.region_count)]
        self.transform_robot_yaw = [[] for _ in range(self.region_count)]
        self.transform_aurora_x = [[] for _ in range(self.region_count)]
        self.transform_aurora_y = [[] for _ in range(self.region_count)]
        self.transform_aurora_z = [[] for _ in range(self.region_count)]
        self.transform_aurora_quat_x = [[] for _ in range(self.region_count)]
        self.transform_aurora_quat_y = [[] for _ in range(self.region_count)]
        self.transform_aurora_quat_z = [[] for _ in range(self.region_count)]
        self.transform_aurora_quat_w = [[] for _ in range(self.region_count)]
        self.transform_sensor_quat_from_robot = [[] for _ in range(self.region_count)]
        
        # Calculate region boundaries
        self.x_boundaries = np.linspace(begin_x, end_x, divisions_per_axis + 1)
        self.y_boundaries = np.linspace(begin_y, end_y, divisions_per_axis + 1)
        self.z_boundaries = np.linspace(begin_z, end_z, divisions_per_axis + 1)

    def get_region_number(self, x, y, z):
        """
        Calculate region number from coordinates. Returns None if out of bounds.
        
        Args:
            x, y, z (float): Coordinates
            
        Returns:
            int or None: Region number or None if out of bounds
        """
        # Tolerance for floating point errors
        epsilon = 5

        # Boundary check with tolerance
        x_ok = (self.x_boundaries[0] - epsilon) <= x <= (self.x_boundaries[-1] + epsilon)
        y_ok = (self.y_boundaries[0] - epsilon) <= y <= (self.y_boundaries[-1] + epsilon)
        z_ok = (self.z_boundaries[0] - epsilon) <= z <= (self.z_boundaries[-1] + epsilon)

        if not (x_ok and y_ok and z_ok):
            print(f"Warning: Coordinates are out of boundary range. Stopping process.")
            print(f"Coordinates: [{x}, {y}, {z}]")
            return None
            
        # Calculate partition from coordinates
        part_x = np.searchsorted(self.x_boundaries[1:-1], float(x))
        part_y = np.searchsorted(self.y_boundaries[1:-1], float(y))
        part_z = np.searchsorted(self.z_boundaries[1:-1], float(z))
        
        return part_x + part_y * self.divisions + part_z * self.divisions ** 2

    def process_data(self, targetData):
        """
        Sort data by region.
        
        Args:
            targetData (np.ndarray): Input data array
            
        Returns:
            tuple: Separated data arrays
        """
        robot_x, robot_y, robot_z = [], [], []
        robot_roll, robot_pitch, robot_yaw = [], [], []
        aurora_x, aurora_y, aurora_z = [], [], []
        aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w = [], [], [], []
          
        for xyz in targetData:
            robot_x.append(xyz[0])
            robot_y.append(xyz[1])
            robot_z.append(xyz[2])
            robot_roll.append(xyz[3])
            robot_pitch.append(xyz[4])
            robot_yaw.append(xyz[5])
            aurora_x.append(xyz[6])
            aurora_y.append(xyz[7])
            aurora_z.append(xyz[8])
            aurora_quat_x.append(xyz[9])
            aurora_quat_y.append(xyz[10])
            aurora_quat_z.append(xyz[11])
            aurora_quat_w.append(xyz[12])
            
            region = self.get_region_number(xyz[0], xyz[1], xyz[2])
            
            self.transform_robot_x[region].append(float(xyz[0]))
            self.transform_robot_y[region].append(float(xyz[1]))
            self.transform_robot_z[region].append(float(xyz[2]))
            self.transform_robot_roll[region].append(float(xyz[3]))
            self.transform_robot_pitch[region].append(float(xyz[4]))
            self.transform_robot_yaw[region].append(float(xyz[5]))
            self.transform_aurora_x[region].append(float(xyz[6]))
            self.transform_aurora_y[region].append(float(xyz[7]))
            self.transform_aurora_z[region].append(float(xyz[8]))
            self.transform_aurora_quat_x[region].append(float(xyz[9]))
            self.transform_aurora_quat_y[region].append(float(xyz[10]))
            self.transform_aurora_quat_z[region].append(float(xyz[11]))
            self.transform_aurora_quat_w[region].append(float(xyz[12]))
            
        return robot_x, robot_y, robot_z, robot_roll, robot_pitch, robot_yaw, aurora_x, aurora_y, aurora_z, aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w

    def calculate_transformations(self):
        """
        Calculate transformation matrices for each region.
        
        Returns:
            tuple: (rotation_matrices, translation_vectors)
        """
        # Import here to avoid circular import
        from .utils import find_transformation
        
        R_aurora_to_robot_matrices = []
        T_aurora_to_robot_vectors = []
        
        for i in range(self.region_count):
            if len(self.transform_robot_x[i]) > 0:
                P1 = np.column_stack((
                    self.transform_robot_x[i],
                    self.transform_robot_y[i],
                    self.transform_robot_z[i]
                ))
                P2 = np.column_stack((
                    self.transform_aurora_x[i],
                    self.transform_aurora_y[i],
                    self.transform_aurora_z[i]
                ))
                
                # find_transformation(P2, P1) returns R, t where P1 = R * P2 + t
                R_matrix, t = find_transformation(P2, P1)
                R_aurora_to_robot_matrices.append(R_matrix)
                T_aurora_to_robot_vectors.append(np.array(t))
            else:
                R_aurora_to_robot_matrices.append(None)
                T_aurora_to_robot_vectors.append(None)
        
        return R_aurora_to_robot_matrices, T_aurora_to_robot_vectors
    
    @staticmethod
    def transform_point_and_orientation(point_before, quaternion_before, translation_vector_A2B, R_matrix_A2B):
        """
        Transform point and orientation from coordinate system A to B.
        
        Args:
            point_before (np.ndarray): Point in coordinate system A (x, y, z)
            quaternion_before (np.ndarray): Orientation in coordinate system A (quaternion)
            translation_vector_A2B (np.ndarray): Translation vector from A to B (x, y, z)
            R_matrix_A2B (np.ndarray): Rotation matrix from A to B (3x3)
            
        Returns:
            tuple: (transformed_point, euler_angles_deg, quaternion)
        """
        # Point transformation: P_B = R * P_A + translation_vector_A2B
        point_after = R_matrix_A2B @ point_before + translation_vector_A2B
        
        # Orientation transformation
        rotation_before = R.from_quat(quaternion_before)
        rotation_A2B = R.from_matrix(R_matrix_A2B)
        
        # For fixed-axis rotation (apply coordinate system B rotation first)
        rotation_after = rotation_A2B * rotation_before

        # Express transformed orientation as Euler angles and quaternion
        euler_after = rotation_after.as_euler('zyx', degrees=True)
        quaternion_after = rotation_after.as_quat()
        
        return point_after, euler_after, quaternion_after

    def transform_coordinates(self, point, quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices):
        """
        Transform coordinates and quaternion using appropriate region transformation.
        
        Args:
            point (list): Point coordinates [x, y, z]
            quaternion (list): Orientation quaternion [x, y, z, w]
            R_aurora_to_robot_matrices (list): List of rotation matrices per region
            T_aurora_to_robot_vectors (list): List of translation vectors per region
            R_sensor_to_arm_matrices (list): List of sensor-to-arm rotation matrices
            
        Returns:
            tuple: Transformed coordinates, Euler angles, quaternions
        """
        # Convert input to numpy arrays
        point_array = np.array(point)
        quaternion_array = np.array(quaternion)
        
        # Calculate transformed coordinates for all valid regions
        valid_points = []
        
        for region in range(self.region_count):
            if R_aurora_to_robot_matrices[region] is not None:
                # Execute coordinate system transformation
                robot_point_checking, _, _ = self.transform_point_and_orientation(
                    point_array,
                    quaternion_array,
                    T_aurora_to_robot_vectors[region],
                    R_aurora_to_robot_matrices[region]
                )
                valid_points.append(robot_point_checking)

        # If no valid transformation results
        if len(valid_points) == 0:
            print("Transformation failed: No valid transformation matrix found")
            return None, None, None, None, None
        
        # Calculate average of all valid transformation results as temporary robot coordinates
        average_point = np.mean(valid_points, axis=0)
        
        # Determine appropriate region number from temporary robot coordinates
        region = self.get_region_number(average_point[0], average_point[1], average_point[2])

        # Check if region is valid
        if region is None:
            # Stop processing if out of range
            return None, None, None, None, None
        
        # Check if transformation matrix for determined region exists
        if R_aurora_to_robot_matrices[region] is not None:
            # Execute final coordinate system transformation
            robot_point_transformed, robot_euler_transformed, robot_quat_transformed = self.transform_point_and_orientation(
                point_array,
                quaternion_array,
                T_aurora_to_robot_vectors[region],
                R_aurora_to_robot_matrices[region]
            )

            if R_sensor_to_arm_matrices is not None:
                # If sensor-to-arm transformation matrix exists, apply additional transformation
                _, arm_euler_transformed, arm_quat_transformed = self.transform_point_and_orientation(
                    robot_point_transformed,
                    robot_quat_transformed,
                    np.zeros(3),  # Translation vector is zero
                    R_sensor_to_arm_matrices[region]
                )
            else:
                # If sensor-to-arm transformation matrix doesn't exist, return robot coordinate system as is
                arm_euler_transformed = robot_euler_transformed
                arm_quat_transformed = robot_quat_transformed

            return robot_point_transformed, robot_euler_transformed, robot_quat_transformed, arm_euler_transformed, arm_quat_transformed
        else:
            # Return None if transformation matrix doesn't exist
            print("Transformation failed: No transformation matrix found for temporary robot coordinates")
            return None, None, None, None, None

    def create_sensor_quat_from_robot(self, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer):
        """
        Create sensor orientation quaternions viewed from robot coordinate system from Aurora orientation.
        """
        for i in range(self.region_count):
            for j in range(len(self.transform_aurora_x[i])):
                # Get Aurora coordinate system point and quaternion
                input_point = np.array([
                    self.transform_aurora_x[i][j],
                    self.transform_aurora_y[i][j],
                    self.transform_aurora_z[i][j]
                ])
                input_quaternion = np.array([
                    self.transform_aurora_quat_x[i][j],
                    self.transform_aurora_quat_y[i][j],
                    self.transform_aurora_quat_z[i][j],
                    self.transform_aurora_quat_w[i][j]
                ])

                _, _, quaternion, _, _ = transformer.transform_coordinates(input_point, input_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices=None)

                # Save sensor orientation quaternion
                self.transform_sensor_quat_from_robot[i].append(quaternion)
    
    def calculate_arm_transformations(self):
        """
        Calculate transformation matrices between robot arm and sensor for each region.
        
        Returns:
            list: List of optimized rotation matrices per region
        """
        # Import here to avoid circular import
        from .utils import estimate_transform_matrix
        
        # Convert Euler angles and quaternions to rotation matrices
        R_matrices_arms = []  # Arm rotation matrix list
        R_matrices_sensors = []  # Sensor rotation matrix list
        R_matrices_sen2arm = []  # Sensor-to-arm rotation matrix list
        
        for i in range(self.region_count):
            # Check if data exists for each region
            if len(self.transform_robot_roll[i]) > 0:
                # Calculate rotation matrix from robot Euler angles for the region
                for r, p, y in zip(self.transform_robot_roll[i], self.transform_robot_pitch[i], self.transform_robot_yaw[i]):
                    rot = R.from_euler('zyx', [r, p, y], degrees=True)
                    R_matrices_arms.append(rot.as_matrix())
                
                # Calculate rotation matrix from sensor quaternions for the region
                for sensor_quat in self.transform_sensor_quat_from_robot[i]:
                    rot = R.from_quat(sensor_quat)
                    R_matrices_sensors.append(rot.as_matrix())
                    
                R_matrix_sen2arm = estimate_transform_matrix(R_matrices_arms, R_matrices_sensors)
                
                R_matrices_sen2arm.append(R_matrix_sen2arm)
            else:
                # Add None for regions without data
                R_matrices_sen2arm.append(None)
        
        return R_matrices_sen2arm 

    @staticmethod
    def rotation_error(quaternion, R_matrices_arm, R_matrices_sensor):
        """
        Calculate transformation error by quaternion.
        
        Args:
            quaternion (array-like): Quaternion to optimize [x, y, z, w]
            R_matrices_arm (list): List of arm orientation rotation matrices from robot coordinate system
            R_matrices_sensor (list): List of sensor orientation rotation matrices from robot coordinate system
        
        Returns:
            np.ndarray: Flattened error between each pair of rotation matrices
        """
        # Normalize quaternion
        quat_normalized = quaternion / np.linalg.norm(quaternion)
        # Create rotation matrix from quaternion
        R_matrix_sen2arm = R.from_quat(quat_normalized).as_matrix()
        
        # Calculate error between each pair of rotation matrices
        errors = []
        for R_matrix_arm, R_matrix_sensor in zip(R_matrices_arm, R_matrices_sensor):
            R_matrix_arm_predicted = R_matrix_sen2arm @ R_matrix_sensor
            
            # Calculate rotation matrix difference
            R_diff = R_matrix_arm.T @ R_matrix_arm_predicted
            angle_error = np.arccos(np.clip((np.trace(R_diff) - 1.0) / 2.0, -1.0, 1.0))
            errors.append(angle_error)
            
        return np.array(errors)