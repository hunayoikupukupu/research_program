import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from utils.transformation_matrix import find_transformation, estimate_transform_matrix

class AdaptiveTransform:
    def __init__(self, divisions_per_axis, begin_x, end_x, begin_y, end_y, begin_z, end_z):  
        """
        divisions_per_axis: 各軸の分割数（2なら8領域、3なら27領域、4なら64領域）
        begin_x, end_x: X軸の範囲
        begin_y, end_y: Y軸の範囲
        begin_z, end_z: Z軸の範囲
        """
        self.divisions = divisions_per_axis
        self.region_count = divisions_per_axis ** 3

        self.begin_x = begin_x
        self.end_x = end_x
        self.begin_y = begin_y
        self.end_y = end_y
        self.begin_z = begin_z
        self.end_z = end_z
        
        # 領域ごとのデータを格納するリストの初期化
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
        
        # 領域の境界値を計算
        self.x_boundaries = np.linspace(begin_x, end_x, divisions_per_axis + 1)
        self.y_boundaries = np.linspace(begin_y, end_y, divisions_per_axis + 1)
        self.z_boundaries = np.linspace(begin_z, end_z, divisions_per_axis + 1)

    def get_region_number(self, x, y, z):
        """座標から領域番号を計算。範囲外の場合はNoneを返す"""
        # 浮動小数点誤差を考慮した許容誤差
        epsilon = 5

        # 境界チェック（許容誤差を含む）
        x_ok = (self.x_boundaries[0] - epsilon) <= x <= (self.x_boundaries[-1] + epsilon)
        y_ok = (self.y_boundaries[0] - epsilon) <= y <= (self.y_boundaries[-1] + epsilon)
        z_ok = (self.z_boundaries[0] - epsilon) <= z <= (self.z_boundaries[-1] + epsilon)

        if not (x_ok and y_ok and z_ok):
            print(f"警告: 座標が境界範囲外です。処理を中止します。")
            print(f"座標: [{x}, {y}, {z}]")
            return None
            
        # 座標から区画を計算
        part_x = np.searchsorted(self.x_boundaries[1:-1], float(x))
        part_y = np.searchsorted(self.y_boundaries[1:-1], float(y))
        part_z = np.searchsorted(self.z_boundaries[1:-1], float(z))
        
        return part_x + part_y * self.divisions + part_z * self.divisions ** 2

    def process_data(self, targetData):
        """データを領域ごとに振り分け"""
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
        """領域ごとの変換行列を計算"""
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
                
                R_matrix, t = find_transformation(P2, P1)
                R_aurora_to_robot_matrices.append(R_matrix)
                T_aurora_to_robot_vectors.append(np.array(t))
            else:
                R_aurora_to_robot_matrices.append(None)
                T_aurora_to_robot_vectors.append(None)
        
        return R_aurora_to_robot_matrices, T_aurora_to_robot_vectors
    
    # 座標変換に使用するメソッド
    @staticmethod
    def transform_point_and_orientation(point_before, quaternion_before, translation_vector_A2B, R_matrix_A2B):
        """
        座標系A上の点と姿勢を座標系Bへ変換する。
        :param point_before: 座標系A上の点 (x, y, z)
        :param quaternion_before: 座標系Aにおける姿勢 (クォータニオン)
        :param translation_vector_A2B: 座標系AからBへの並進ベクトル (x, y, z)
        :param R_matrix_A2B: 座標系AからBへの回転行列 (3x3)
        :return: (座標系Bにおける点の座標, オイラー角[度], クォータニオン)
        """
        # 点の座標変換: P_B = R * P_A + translation_vector_A2B
        point_after = R_matrix_A2B @ point_before + translation_vector_A2B
        
        # 姿勢の変換
        rotation_before = R.from_quat(quaternion_before)
        rotation_A2B = R.from_matrix(R_matrix_A2B)
        
        # 固定軸回転の場合（座標系Bの回転を先に適用）
        rotation_after = rotation_A2B * rotation_before

        # 変換後の姿勢をオイラー角とクォータニオンで表現
        euler_after = rotation_after.as_euler('XYZ', degrees=True)
        quaternion_after = rotation_after.as_quat()
        
        return point_after, euler_after, quaternion_after

    def transform_coordinates(self, point, quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices):
        """
        座標とクォータニオンを受け取り、適切な領域の変換を適用する
        
        :param point: 変換する点の座標 [x, y, z]
        :param quaternion: 姿勢を表すクォータニオン [x, y, z, w]
        :param R_aurora_to_robot_matrices: 領域ごとの回転行列のリスト
        :param T_aurora_to_robot_vectors: 領域ごとの並進ベクトルのリスト
        :param R_sensor_to_arm_matrices: センサーからアームへの回転行列のリスト
        :return: sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot
                変換後の座標、オイラー角、クォータニオンを返す。
                変換できない場合はNoneを返す
        """
        
        # 入力をnumpy配列に変換
        point_array = np.array(point)
        quaternion_array = np.array(quaternion)
        
        # 全ての有効な領域における変換後の座標を計算
        valid_points = []
        
        for region in range(self.region_count):
            if R_aurora_to_robot_matrices[region] is not None:
                # 座標系変換を実行
                robot_point_checking, _, _ = self.transform_point_and_orientation(
                    point_array,
                    quaternion_array,
                    T_aurora_to_robot_vectors[region],
                    R_aurora_to_robot_matrices[region]
                )
                valid_points.append(robot_point_checking)

        # 有効な変換結果がない場合
        if len(valid_points) == 0:
            print("変換失敗: 有効な変換行列が見つかりませんでした")
            return None, None, None, None, None
        
        # 全ての有効な変換結果の平均を計算して仮のロボット座標とする
        average_point = np.mean(valid_points, axis=0)
        
        # 仮のロボット座標から適切な領域番号を決定
        region = self.get_region_number(average_point[0], average_point[1], average_point[2])

        # 中間処理の情報をログ出力（デバッグ用）
        # print(f"仮のロボット座標による領域判定: 座標 [{average_point[0]:.4f}, {average_point[1]:.4f}, {average_point[2]:.4f}], 領域番号 {region}")

        # 有効な領域かチェック
        if region is None:
            # 範囲外の場合は処理を終了
            return None, None, None, None, None
        
        # 決定した領域の変換行列が存在するか確認
        if R_aurora_to_robot_matrices[region] is not None:

            # print(f"R_aurora_to_robot_matrices[{region}]:\n{R_aurora_to_robot_matrices[region]}")
            # print(f"T_aurora_to_robot_vectors[{region}]:\n{T_aurora_to_robot_vectors[region]}")
            # 最終的な座標系変換を実行
            robot_point_transformed, robot_euler_transformed, robot_quat_transformed = self.transform_point_and_orientation(
                point_array,
                quaternion_array,
                T_aurora_to_robot_vectors[region],
                R_aurora_to_robot_matrices[region]
            )

            if R_sensor_to_arm_matrices is not None:
                # センサーからアームへの変換行列が存在する場合は、さらに変換を適用
                _, arm_euler_transformed, arm_quat_transformed = self.transform_point_and_orientation(
                    robot_point_transformed,
                    robot_quat_transformed,
                    np.zeros(3),  # 並進ベクトルはゼロ
                    R_sensor_to_arm_matrices[region]
                )
            else:
                # センサーからアームへの変換行列が存在しない場合は、ロボット座標系のまま返す
                arm_euler_transformed = robot_euler_transformed
                arm_quat_transformed = robot_quat_transformed

            return robot_point_transformed, robot_euler_transformed, robot_quat_transformed, arm_euler_transformed, arm_quat_transformed
        else:
            # 変換行列が存在しない場合はNoneを返す
            print("変換失敗: 仮のロボット座標に対応する変換行列が見つかりませんでした")
            return None, None, None, None, None

    # 姿勢変換を計算するメソッド
    def create_sensor_quat_from_robot(self, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer):
        """
        ロボット座標系から見たセンサー姿勢のクォータニオンをAurora姿勢（クォータニオン）から作成するメソッド
        """
        for i in range(self.region_count):

            for j in range(len(self.transform_aurora_x[i])):
                # Aurora座標系の点とクォータニオンを取得
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

                # センサー姿勢のクォータニオンを保存
                self.transform_sensor_quat_from_robot[i].append(quaternion)
                # print(f"領域 {i} のセンサー姿勢のクォータニオンを計算しました。")
                # print(f"transform_sensor_quat_from_robot[{i}][{j}]: {quaternion}")
    
    def calculate_arm_transformations(self):
        """
        領域ごとのロボットアームとセンサー間の変換行列を計算
        
        Returns:
        --------
        R_matrices_sen2arm : list
            領域ごとの最適化された回転行列のリスト
        """
        # オイラー角とクォータニオンを回転行列に変換
        R_matrices_arms = []  # アームの回転行列リスト
        R_matrices_sensors = []  # センサーの回転行列リスト
        R_matrices_sen2arm = []  # センサーからアームへの回転行列リスト
        
        for i in range(self.region_count):
            # 各領域のデータが存在するかチェック
            if len(self.transform_robot_roll[i]) > 0:
                # 該当領域のロボットのオイラー角から回転行列を計算
                for r, p, y in zip(self.transform_robot_roll[i], self.transform_robot_pitch[i], self.transform_robot_yaw[i]):
                    rot = R.from_euler('XYZ', [r, p, y], degrees=True)
                    R_matrices_arms.append(rot.as_matrix())
                    # print(f"roll: {r}, pitch: {p}, yaw: {y} -> R_matrix_arm: {rot.as_matrix()}")
                
                # 該当領域のセンサーのクォータニオンから回転行列を計算
                for sensor_quat in self.transform_sensor_quat_from_robot[i]:
                    rot = R.from_quat(sensor_quat)
                    R_matrices_sensors.append(rot.as_matrix())
                    # print(f"sensor_quat: {sensor_quat} -> R_matrix_sensor: {rot.as_matrix()}")
                R_matrix_sen2arm = estimate_transform_matrix(R_matrices_arms, R_matrices_sensors)
                
                R_matrices_sen2arm.append(R_matrix_sen2arm)
                # print(f"領域 {i} のセンサーからアームへの回転行列を計算しました。")
                # print(f"R_matrices_sen2arm[{i}]:\n{R_matrix_sen2arm}")
            else:
                # データが存在しない領域にはNoneを追加
                R_matrices_sen2arm.append(None)
        
        return R_matrices_sen2arm 

    @staticmethod
    def rotation_error(quaternion, R_matrices_arm, R_matrices_sensor):
        """
        クォータニオンによる変換の誤差を計算
        
        Parameters:
        -----------
        quaternion : array-like
            最適化するクォータニオン [x, y, z, w]
        R_matrices_arm : list
            ロボット座標系から見たアーム姿勢の回転行列のリスト
        R_matrices_sensor : list
            ロボット座標系から見たセンサー姿勢の回転行列のリスト
        
        Returns:
        --------
        errors : array
            各ペアの回転行列間の誤差を平坦化したもの
        """
        # クォータニオンを正規化
        quat_normalized = quaternion / np.linalg.norm(quaternion)
        # クォータニオンから回転行列を作成
        R_matrix_sen2arm = R.from_quat(quat_normalized).as_matrix()
        
        # 各ペアの回転行列間の誤差を計算
        errors = []
        for R_matrix_arm, R_matrix_sensor in zip(R_matrices_arm, R_matrices_sensor):
            R_matrix_arm_predicted = R_matrix_sen2arm @ R_matrix_sensor
            
            # 回転行列の差分を計算
            R_diff = R_matrix_arm.T @ R_matrix_arm_predicted
            angle_error = np.arccos(np.clip((np.trace(R_diff) - 1.0) / 2.0, -1.0, 1.0))
            errors.append(angle_error)
            
        return np.array(errors)

def build_coordinate_transformation_model(x_range, y_range, z_range, divisions, data_file):
    """
    座標変換モデルを構築する関数
    
    :param x_range: X軸の範囲 (開始値, 終了値)
    :param y_range: Y軸の範囲 (開始値, 終了値)
    :param z_range: Z軸の範囲 (開始値, 終了値)
    :param divisions: 分割数（各軸divisions分割でdivisions^3領域）
    :param data_file: データファイルのパス
    :return: (R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer) 回転行列、並進ベクトル、変換オブジェクト
    """
    # 範囲を展開
    begin_x, end_x = x_range
    begin_y, end_y = y_range
    begin_z, end_z = z_range
    
    # パラメータの表示
    print(f"モデル構築パラメータ:")
    print(f"  分割数: {divisions} ({divisions}^3 = {divisions**3}領域)")
    print(f"  X軸範囲: {begin_x} から {end_x}")
    print(f"  Y軸範囲: {begin_y} から {end_y}")
    print(f"  Z軸範囲: {begin_z} から {end_z}")
    print(f"  データファイル: {data_file}")
    
    # CSVファイルの読み込み
    try:
        targetData = np.loadtxt(data_file, skiprows=1, delimiter=',')
    except Exception as e:
        print(f"エラー: データファイルの読み込みに失敗しました。 {e}")
        return None, None, None
    
    # AdaptiveTransformクラスの初期化
    transformer = AdaptiveTransform(divisions, begin_x, end_x, begin_y, end_y, begin_z, end_z)
    
    # データの処理
    robot_x, robot_y, robot_z, robot_roll, robot_pitch, robot_yaw, aurora_x, aurora_y, aurora_z, aurora_quat_x, aurora_quat_y, aurora_quat_z, aurora_quat_w = transformer.process_data(targetData)
    
    # 変換行列の計算
    R_aurora_to_robot_matrices, T_aurora_to_robot_vectors = transformer.calculate_transformations()
    
    # 変換行列の作成結果を表示
    successful_regions = sum(1 for matrix in R_aurora_to_robot_matrices if matrix is not None)
    print(f"回転行列と並進ベクトルの作成結果: {successful_regions}/{len(R_aurora_to_robot_matrices)}領域で成功")
    
    return R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer


def transform_pose(input_point, input_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices, transformer):
    """
    指定された座標と姿勢を変換する関数
    
    :param input_point: 変換する座標 [x, y, z]
    :param input_quaternion: 変換する姿勢のクォータニオン [x, y, z, w]
    :param R_aurora_to_robot_matrices: 回転行列のリスト
    :param T_aurora_to_robot_vectors: 並進ベクトルのリスト
    :param transformer: AdaptiveTransformオブジェクト
    :return: (sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot) 変換後の座標、オイラー角、クォータニオン
    """
    # 座標変換の実行
    sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot = transformer.transform_coordinates(
        input_point,
        input_quaternion,
        R_aurora_to_robot_matrices,
        T_aurora_to_robot_vectors,
        R_sensor_to_arm_matrices
    )
    
    # 変換結果の表示
    print(f"変換前(Sensor_from_Aurora): 座標 [{input_point[0]}, {input_point[1]}, {input_point[2]}], クォータニオン [{input_quaternion[0]}, {input_quaternion[1]}, {input_quaternion[2]}, {input_quaternion[3]}]")

    if sensor_point_from_robot is not None:
        # 小数点第4位で四捨五入
        rounded_sensor_point = np.round(sensor_point_from_robot, 4)
        rounded_sensor_euler = np.round(sensor_euler_from_robot, 4)
        rounded_sensor_quat = np.round(sensor_quat_from_robot, 4)
        print(f"変換後(Sensor_from_Robot): 座標 [{rounded_sensor_point[0]:.4f}, {rounded_sensor_point[1]:.4f}, {rounded_sensor_point[2]:.4f}], オイラー角 [{rounded_sensor_euler[0]:.4f}, {rounded_sensor_euler[1]:.4f}, {rounded_sensor_euler[2]:.4f}], クォータニオン [{rounded_sensor_quat[0]:.4f}, {rounded_sensor_quat[1]:.4f}, {rounded_sensor_quat[2]:.4f}, {rounded_sensor_quat[3]:.4f}]")
    else:
        print("変換失敗: 指定された座標に対応する変換行列が見つかりませんでした")

    if arm_euler_from_robot is not None:
        # 小数点第4位で四捨五入
        rounded_arm_euler = np.round(arm_euler_from_robot, 4)
        rounded_arm_quat = np.round(arm_quat_from_robot, 4)
        print(f"変換後(Arm_from_Robot): オイラー角 [{rounded_arm_euler[0]:.4f}, {rounded_arm_euler[1]:.4f}, {rounded_arm_euler[2]:.4f}], クォータニオン [{rounded_arm_quat[0]:.4f}, {rounded_arm_quat[1]:.4f}, {rounded_arm_quat[2]:.4f}, {rounded_arm_quat[3]:.4f}]")
    
    return sensor_point_from_robot, sensor_euler_from_robot, sensor_quat_from_robot, arm_euler_from_robot, arm_quat_from_robot

def print_transformation_results(R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices):
    """
    変換結果を表示する関数
    :param R_aurora_to_robot_matrices: Auroraからロボットへの回転行列のリスト
    :param T_aurora_to_robot_vectors: Auroraからロボットへの並進ベクトルのリスト
    :param R_sensor_to_arm_matrices: センサーからアームへの回転行列のリスト
    """
    print("R_aurora_to_robot_matrices (最初の1つ):")
    if len(R_aurora_to_robot_matrices) > 0:
        matrix = R_aurora_to_robot_matrices[0]
        for row in matrix:
            print(f"  [{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")
        # オイラー角への変換（固定軸回転）
        euler_xyz = R.from_matrix(matrix).as_euler('XYZ', degrees=True)
        print(f"  オイラー角(XYZ, deg): [{euler_xyz[0]:7.2f}, {euler_xyz[1]:7.2f}, {euler_xyz[2]:7.2f}]")
    else:
        print("  配列が空です")

    print("\nT_aurora_to_robot_vectors (最初の1つ):")
    if len(T_aurora_to_robot_vectors) > 0:
        vector = T_aurora_to_robot_vectors[0]
        print(f"  [{vector[0]:8.2f}, {vector[1]:8.2f}, {vector[2]:8.2f}]")
    else:
        print("  配列が空です")

    print("\nR_sensor_to_arm_matrices (最初の1つ):")
    if len(R_sensor_to_arm_matrices) > 0:
        matrix = R_sensor_to_arm_matrices[0]
        for row in matrix:
            print(f"  [{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")
        # オイラー角への変換（固定軸回転）
        euler_xyz = R.from_matrix(matrix).as_euler('XYZ', degrees=True)
        print(f"  オイラー角(XYZ, deg): [{euler_xyz[0]:7.2f}, {euler_xyz[1]:7.2f}, {euler_xyz[2]:7.2f}]")
    else:
        print("  配列が空です")

def main(x_range, y_range, z_range, divisions, data_file, input_point=None, input_quaternion=None):
    """
    メイン処理を実行する関数
    
    :param x_range: X軸の範囲 (開始値, 終了値)
    :param y_range: Y軸の範囲 (開始値, 終了値)
    :param z_range: Z軸の範囲 (開始値, 終了値)
    :param divisions: 分割数（各軸divisions分割でdivisions^3領域）
    :param data_file: データファイルのパス
    :param input_point: 変換する座標 [x, y, z] (デフォルト: None)
    :param input_quaternion: 変換する姿勢のクォータニオン [x, y, z, w] (デフォルト: None)
    :return: 変換結果のタプル (変換後の座標, 変換後のオイラー角, 変換後のクォータニオン) または None
    """
    # 変換モデルの構築
    R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer = build_coordinate_transformation_model(
        x_range, y_range, z_range, divisions, data_file
    )
    
    if R_aurora_to_robot_matrices is None:
        return None
    
    # センサーのAurora姿勢をロボット姿勢に変換するメソッドを作成
    transformer.create_sensor_quat_from_robot(R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer)

    # アーム座標系からセンサー座標系への変換行列を計算
    R_sensor_to_arm_matrices = transformer.calculate_arm_transformations()

    # 変換結果を表示
    print_transformation_results(R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices)

    # 入力が提供されている場合は変換を実行
    if input_point is not None and input_quaternion is not None:
        return transform_pose(input_point, input_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices, transformer)
    else:
        # 入力が提供されていない場合はデフォルトの例を使用
        example_point = [250, 0, 100]  # 変換する点の座標
        example_quaternion = [0, 0, 0, 1]  # 姿勢を表すクォータニオン [x, y, z, w]

        transform_pose(example_point, example_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, R_sensor_to_arm_matrices, transformer)
        
        # 例の変換結果を返さない（None）
        return None    


if __name__ == "__main__":
    # ここで全てのパラメータを一か所で設定（ここだけを変更すれば良い）
    result = main(
        x_range=(250, 350),                    # X座標の範囲 (開始値, 終了値)
        y_range=(-50, 50),                     # Y座標の範囲 (開始値, 終了値)
        z_range=(75, 175),                     # Z座標の範囲 (開始値, 終了値)
        divisions=2,                           # 分割数（各軸divisions分割でdivisions^3領域）
        data_file='robot&aurora/current_code/calibration_data/pose_R0-0-45_T100-0-0_ARM180-0-0_SEN0-0-45_n0_qn0.csv',  # データファイルのパス
        input_point=[70.71068,-141.42136,95.0],             # 変換する点の座標 [x, y, z]
        input_quaternion=[0.48296,-0.22414,0.12941,0.83652]          # 姿勢を表すクォータニオン [x, y, z, w]
    )