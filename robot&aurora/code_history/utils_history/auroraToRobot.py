import numpy as np
from scipy.spatial.transform import Rotation as R

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
from utils.transformation_matrix import findTransformation


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
        self.transform_aurora_x = [[] for _ in range(self.region_count)]
        self.transform_aurora_y = [[] for _ in range(self.region_count)]
        self.transform_aurora_z = [[] for _ in range(self.region_count)]
        
        # 領域の境界値を計算
        self.x_boundaries = np.linspace(begin_x, end_x, divisions_per_axis + 1)
        self.y_boundaries = np.linspace(begin_y, end_y, divisions_per_axis + 1)
        self.z_boundaries = np.linspace(begin_z, end_z, divisions_per_axis + 1)

    def get_region_number(self, x, y, z):
        """座標から領域番号を計算"""
        part_x = np.searchsorted(self.x_boundaries[1:-1], float(x))
        part_y = np.searchsorted(self.y_boundaries[1:-1], float(y))
        part_z = np.searchsorted(self.z_boundaries[1:-1], float(z))
        
        return part_x + part_y * self.divisions + part_z * self.divisions ** 2

    def process_data(self, targetData):
        """データを領域ごとに振り分け"""
        x, y, z = [], [], []
        aurora_x, aurora_y, aurora_z = [], [], []
        
        for xyz in targetData:
            x.append(xyz[0])
            y.append(xyz[1])
            z.append(xyz[2])
            aurora_x.append(xyz[3])
            aurora_y.append(xyz[4])
            aurora_z.append(xyz[5])
            
            region = self.get_region_number(xyz[0], xyz[1], xyz[2])
            
            self.transform_robot_x[region].append(float(xyz[0]))
            self.transform_robot_y[region].append(float(xyz[1]))
            self.transform_robot_z[region].append(float(xyz[2]))
            self.transform_aurora_x[region].append(float(xyz[3]))
            self.transform_aurora_y[region].append(float(xyz[4]))
            self.transform_aurora_z[region].append(float(xyz[5]))
            
        return x, y, z, aurora_x, aurora_y, aurora_z

    def calculate_transformations(self):
        """領域ごとの変換行列を計算"""
        R_matrices = []
        t_vectors = []
        
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
                
                R_matrix, t = findTransformation(P2, P1)
                R_matrices.append(R_matrix)
                t_vectors.append(np.array(t))
                
                print(f"Region {i} transformation:")
                print("Translation vector:\n", np.round(t, 4))
                print("Rotation matrix:\n", np.round(R_matrix, 4))
                print(f"Number of points in region: {len(self.transform_robot_x[i])}")
            else:
                R_matrices.append(None)
                t_vectors.append(None)
                print(f"Region {i}: No data points")
        
        return R_matrices, t_vectors
    
    @staticmethod
    def transform_point_and_orientation(point_A, quaternion_A, t, R_matrix):
        """
        座標系A上の点と姿勢を座標系Bへ変換する。
        :param point_A: 座標系A上の点 (x, y, z)
        :param quaternion_A: 座標系Aにおける姿勢 (クォータニオン)
        :param t: 座標系AからBへの並進ベクトル (x, y, z)
        :param R_matrix: 座標系AからBへの回転行列 (3x3)
        :return: (座標系Bにおける点の座標, オイラー角[度], クォータニオン)
        """
        # 点の座標変換: P_B = R * P_A + t
        transformed_point = R_matrix @ point_A + t
        
        # 姿勢の変換
        rotation_A = R.from_quat(quaternion_A)
        rotation_B = R.from_matrix(R_matrix)
        
        # 姿勢の合成を逆順に行う
        rotation_new = rotation_A * rotation_B
        
        # 変換後の姿勢をオイラー角とクォータニオンで表現
        transformed_orientation_euler = rotation_new.as_euler('XYZ', degrees=True)
        transformed_orientation_quat = rotation_new.as_quat()
        
        return transformed_point, transformed_orientation_euler, transformed_orientation_quat

    def transform_coordinates(self, point, quaternion, R_matrices, t_vectors):
        """
        座標とクォータニオンを受け取り、対応する領域の変換を適用する
        
        :param point: 変換する点の座標 [x, y, z]
        :param quaternion: 姿勢を表すクォータニオン [x, y, z, w]
        :param R_matrices: 領域ごとの回転行列のリスト
        :param t_vectors: 領域ごとの並進ベクトルのリスト
        :return: (変換後の座標, 変換後のオイラー角, 変換後のクォータニオン)
                変換できない場合はNoneを返す
        """
        from scipy.spatial.transform import Rotation as R
        
        # 入力をnumpy配列に変換
        point_array = np.array(point)
        quaternion_array = np.array(quaternion)
        
        # 領域番号を取得
        region = self.get_region_number(point[0], point[1], point[2])
        
        # その領域に変換行列が存在するか確認
        if R_matrices[region] is not None:
            # 座標系変換を実行
            transformed_point, transformed_euler, transformed_quat = self.transform_point_and_orientation(
                point_array,
                quaternion_array,
                t_vectors[region],
                R_matrices[region]
            )
            
            return transformed_point, transformed_euler, transformed_quat
        else:
            # 変換行列が存在しない場合はNoneを返す
            return None, None, None

def main():
    # CSVファイルの読み込み
    targetData = np.loadtxt('robot&aurora/20250214/transform_data.csv', skiprows=1, delimiter=',')
    
    # 分割数の設定（2=8領域、3=27領域、4=64領域）
    divisions = 3  # ここで分割数を変更できます
    
    # 可動範囲の設定
    begin_x = 200
    end_x = 300
    begin_y = -50
    end_y = 50
    begin_z = 80
    end_z = 180
    
    # AdaptiveTransformクラスの初期化
    transformer = AdaptiveTransform(divisions, begin_x, end_x, begin_y, end_y, begin_z, end_z)
    
    # データの処理
    x, y, z, aurora_x, aurora_y, aurora_z = transformer.process_data(targetData)
    
    # 変換行列の計算
    R_matrices, t_vectors = transformer.calculate_transformations()
    
    # 座標・姿勢変換の例
    example_point = [250, 0, 100]  # 変換する点の座標
    example_quaternion = [0, 0, 0, 1]  # 姿勢を表すクォータニオン [x, y, z, w]
    transformed_point, transformed_euler, transformed_quat = transformer.transform_coordinates(
        example_point,
        example_quaternion,
        R_matrices,
        t_vectors
    )
    print("Transformed Point:", transformed_point)
    print("Transformed Euler Angles:", transformed_euler)
    print("Transformed Quaternion:", transformed_quat)

if __name__ == "__main__":
    main()