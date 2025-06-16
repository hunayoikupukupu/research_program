import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares


# このプログラムの流れ
# 1. csvファイルからデータを読み込む
# 2. build_coordinate_transformation_modelでAurora=>robot変換モデルを構築する
# 3. transform_poseでセンサーのAurora姿勢をrobot姿勢に変換し「センサー姿勢（ロボット）」配列を作成する
# 4. csvファイルから「ロボットアーム姿勢」を抽出する
# 5. 「ロボットアーム姿勢」と「センサー姿勢（ロボット）」から回転行列R_sensor_to_armを作成する
# 6. サンプルのセンサー姿勢（Aurora）を渡すとそれに対応するロボットアーム姿勢を返すようにする

import numpy as np
from scipy.spatial.transform import Rotation as R
from utils.transformation_matrix import find_transformation

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

        
        # 領域の境界値を計算
        self.x_boundaries = np.linspace(begin_x, end_x, divisions_per_axis + 1)
        self.y_boundaries = np.linspace(begin_y, end_y, divisions_per_axis + 1)
        self.z_boundaries = np.linspace(begin_z, end_z, divisions_per_axis + 1)

    def get_region_number(self, x, y, z):
        """座標から領域番号を計算。範囲外の場合はNoneを返す"""
        # 座標が境界範囲内かチェック
        if (x < self.x_boundaries[0] or x > self.x_boundaries[-1] or
            y < self.y_boundaries[0] or y > self.y_boundaries[-1] or
            z < self.z_boundaries[0] or z > self.z_boundaries[-1]):
            return None  # 範囲外の場合はNoneを返す
            
        # 座標から区画を計算
        part_x = np.searchsorted(self.x_boundaries[1:-1], float(x))
        part_y = np.searchsorted(self.y_boundaries[1:-1], float(y))
        part_z = np.searchsorted(self.z_boundaries[1:-1], float(z))
        
        # 領域番号を計算して返す
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

    @staticmethod
    def rotation_error(quaternion, robot_rotations, sensor_rotations):
        """
        クォータニオンによる変換の誤差を計算
        
        Parameters:
        -----------
        quaternion : array-like
            最適化するクォータニオン [x, y, z, w]
        robot_rotations : list
            ロボットの回転行列のリスト
        sensor_rotations : list
            センサーの回転行列のリスト
        
        Returns:
        --------
        errors : array
            各ペアの回転行列間の誤差を平坦化したもの
        """
        # クォータニオンを正規化
        quat_normalized = quaternion / np.linalg.norm(quaternion)
        
        # クォータニオンから回転行列を作成
        R_transform = R.from_quat(quat_normalized).as_matrix()
        
        # 各ペアの回転行列間の誤差を計算
        errors = []
        for R_robot, R_sensor in zip(robot_rotations, sensor_rotations):
            # 変換後の行列: R_transform @ R_robot は R_sensor に近いはず
            predicted_rotation = R_transform @ R_robot
            
            # フロベニウスノルムで誤差を計算 (行列の差の二乗和の平方根)
            error = R_sensor - predicted_rotation
            errors.extend(error.flatten())  # 1次元配列に平坦化
            
        return np.array(errors)

    def calculate_arm_transformation(self):
        """
        ロボットアームとセンサー間の変換行列を計算
        
        Returns:
        --------
        R_optimal : ndarray
            最適化された回転行列
        """
        # ロボットのオイラー角から回転行列を計算
        arm_rotations = []
        for r, p, y in zip(self.robot_roll, self.robot_pitch, self.robot_yaw):
            rot = R.from_euler('XYZ', [r, p, y])
            arm_rotations.append(rot.as_matrix())
        
        # センサーのクォータニオンから回転行列を計算
        sensor_rotations = []
        for x, y, z, w in zip(self.transform_aurora_quat_x, self.transform_aurora_quat_y, self.transform_aurora_quat_z, self.transform_aurora_quat_w):
            rot = R.from_quat([x, y, z, w])
            sensor_rotations.append(rot.as_matrix())
        
        # 初期推定値（単位回転）
        initial_params = np.array([0, 0, 0, 1])  # クォータニオン [x, y, z, w]
        
        # 最小二乗法で最適化
        result = least_squares(
            AdaptiveTransform.rotation_error, 
            initial_params, 
            args=(arm_rotations, sensor_rotations)
        )
        
        # 最適化されたパラメータから回転行列を取得
        optimal_quat = result.x / np.linalg.norm(result.x)  # 正規化
        R_optimal = R.from_quat(optimal_quat).as_matrix()
        
        return R_optimal
    
    @staticmethod
    def transform_point_and_orientation(point_aurora, quaternion_aurora, translation_vector_aur2rob, R_matrix_aur2rob):
        """
        座標系A上の点と姿勢を座標系Bへ変換する。
        :param point_aurora: 座標系A上の点 (x, y, z)
        :param quaternion_aurora: 座標系Aにおける姿勢 (クォータニオン)
        :param translation_vector_aur2rob: 座標系AからBへの並進ベクトル (x, y, z)
        :param R_matrix_aur2rob: 座標系AからBへの回転行列 (3x3)
        :return: (座標系Bにおける点の座標, オイラー角[度], クォータニオン)
        """
        # 点の座標変換: P_B = R * P_A + translation_vector_aur2rob
        point_robot = R_matrix_aur2rob @ point_aurora + translation_vector_aur2rob
        
        # 姿勢の変換
        rotation_aurora = R.from_quat(quaternion_aurora)
        rotation_aur2rob = R.from_matrix(R_matrix_aur2rob)
        
        # 固定軸回転の場合（座標系Bの回転を先に適用）
        rotation_robot = rotation_aur2rob * rotation_aurora
        
        # 変換後の姿勢をオイラー角とクォータニオンで表現
        transformed_orientation_euler = rotation_robot.as_euler('XYZ', degrees=True)
        transformed_orientation_quat = rotation_robot.as_quat()
        
        return point_robot, transformed_orientation_euler, transformed_orientation_quat

    def transform_coordinates(self, point, quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors):
        """
        座標とクォータニオンを受け取り、適切な領域の変換を適用する
        
        :param point: 変換する点の座標 [x, y, z]
        :param quaternion: 姿勢を表すクォータニオン [x, y, z, w]
        :param R_aurora_to_robot_matrices: 領域ごとの回転行列のリスト
        :param T_aurora_to_robot_vectors: 領域ごとの並進ベクトルのリスト
        :return: (変換後の座標, 変換後のオイラー角, 変換後のクォータニオン)
                変換できない場合はNoneを返す
        """
        from scipy.spatial.transform import Rotation as R
        
        # 入力をnumpy配列に変換
        point_array = np.array(point)
        quaternion_array = np.array(quaternion)
        
        # 全ての有効な領域における変換後の座標を計算
        valid_points = []
        
        for region in range(self.region_count):
            if R_aurora_to_robot_matrices[region] is not None:
                # 座標系変換を実行
                transformed_point, _, _ = self.transform_point_and_orientation(
                    point_array,
                    quaternion_array,
                    T_aurora_to_robot_vectors[region],
                    R_aurora_to_robot_matrices[region]
                )
                valid_points.append(transformed_point)
        
        # 有効な変換結果がない場合
        if len(valid_points) == 0:
            print("変換失敗: 有効な変換行列が見つかりませんでした")
            return None, None, None
        
        # 全ての有効な変換結果の平均を計算して仮のロボット座標とする
        average_point = np.mean(valid_points, axis=0)
        
        # 仮のロボット座標から適切な領域番号を決定
        region = self.get_region_number(average_point[0], average_point[1], average_point[2])

        # 中間処理の情報をログ出力（デバッグ用）
        print(f"仮のロボット座標による領域判定: 座標 [{average_point[0]:.4f}, {average_point[1]:.4f}, {average_point[2]:.4f}], 領域番号 {region}")

        # 有効な領域かチェック
        if region is None:
            # 範囲外の場合は処理を終了
            print("警告: 座標が境界範囲外です。処理を中止します。")
            return None, None, None
        
        # 決定した領域の変換行列が存在するか確認
        if R_aurora_to_robot_matrices[region] is not None:

            print(f"R_aurora_to_robot_matrices[{region}]:\n{R_aurora_to_robot_matrices[region]}")
            print(f"T_aurora_to_robot_vectors[{region}]:\n{T_aurora_to_robot_vectors[region]}")
            # 最終的な座標系変換を実行
            transformed_point, transformed_euler, transformed_quat = self.transform_point_and_orientation(
                point_array,
                quaternion_array,
                T_aurora_to_robot_vectors[region],
                R_aurora_to_robot_matrices[region]
            )
            
            return transformed_point, transformed_euler, transformed_quat
        else:
            # 変換行列が存在しない場合はNoneを返す
            print("変換失敗: 仮のロボット座標に対応する変換行列が見つかりませんでした")
            return None, None, None

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


def transform_pose(input_point, input_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer):
    """
    指定された座標と姿勢を変換する関数
    
    :param input_point: 変換する座標 [x, y, z]
    :param input_quaternion: 変換する姿勢のクォータニオン [x, y, z, w]
    :param R_aurora_to_robot_matrices: 回転行列のリスト
    :param T_aurora_to_robot_vectors: 並進ベクトルのリスト
    :param transformer: AdaptiveTransformオブジェクト
    :return: (transformed_point, transformed_euler, transformed_quat) 変換後の座標、オイラー角、クォータニオン
    """
    # 座標変換の実行
    transformed_point, transformed_euler, transformed_quat = transformer.transform_coordinates(
        input_point,
        input_quaternion,
        R_aurora_to_robot_matrices,
        T_aurora_to_robot_vectors
    )
    
    # 変換結果の表示
    print(f"変換前: 座標 [{input_point[0]}, {input_point[1]}, {input_point[2]}], クォータニオン [{input_quaternion[0]}, {input_quaternion[1]}, {input_quaternion[2]}, {input_quaternion[3]}]")
    
    if transformed_point is not None:
        # 小数点第4位で四捨五入
        rounded_point = np.round(transformed_point, 4)
        rounded_euler = np.round(transformed_euler, 4)
        rounded_quat = np.round(transformed_quat, 4)
        # 回転順序をZYXから表示用のXYZに変換
        print(f"変換後: 座標 [{rounded_point[0]:.4f}, {rounded_point[1]:.4f}, {rounded_point[2]:.4f}], オイラー角 [{rounded_euler[0]:.4f}, {rounded_euler[1]:.4f}, {rounded_euler[2]:.4f}], クォータニオン [{rounded_quat[0]:.4f}, {rounded_quat[1]:.4f}, {rounded_quat[2]:.4f}, {rounded_quat[3]:.4f}]")
    else:
        print("変換失敗: 指定された座標に対応する変換行列が見つかりませんでした")
    
    return transformed_point, transformed_euler, transformed_quat

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
    
    # 入力が提供されている場合は変換を実行
    if input_point is not None and input_quaternion is not None:
        return transform_pose(input_point, input_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer)
    else:
        # 入力が提供されていない場合はデフォルトの例を使用
        example_point = [250, 0, 100]  # 変換する点の座標
        example_quaternion = [0, 0, 0, 1]  # 姿勢を表すクォータニオン [x, y, z, w]
        
        transform_pose(example_point, example_quaternion, R_aurora_to_robot_matrices, T_aurora_to_robot_vectors, transformer)
        
        # 例の変換結果を返さない（None）
        return None    


if __name__ == "__main__":
    # ここで全てのパラメータを一か所で設定（ここだけを変更すれば良い）
    result = main(
        x_range=(250, 350),                    # X座標の範囲 (開始値, 終了値)
        y_range=(-50, 50),                     # Y座標の範囲 (開始値, 終了値)
        z_range=(75, 175),                     # Z座標の範囲 (開始値, 終了値)
        divisions=2,                           # 分割数（各軸divisions分割でdivisions^3領域）
        data_file='robot&aurora/current_code/calibration_data/aurora_robot_pose_log_complex.csv',  # データファイルのパス
        input_point=[11.383886315357232,-4.694703169813527,-198.0362767134],             # 変換する点の座標 [x, y, z]
        input_quaternion=[0, 0, 0, 1]          # 姿勢を表すクォータニオン [x, y, z, w]
    )