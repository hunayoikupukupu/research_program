import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.ticker as mticker
from os.path import dirname, abspath
import sys
parent_dir = dirname(dirname(abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
from utils.transformation_matrix import findTransformation
from utils.coordinate3 import transform_point_and_orientation

class AdaptiveTransform:
    def __init__(self, divisions_per_axis):
        """
        divisions_per_axis: 各軸の分割数（2なら8領域、3なら27領域、4なら64領域）
        """
        self.divisions = divisions_per_axis
        self.region_count = divisions_per_axis ** 3
        
        # 領域ごとのデータを格納するリストの初期化
        self.transform_robot_x = [[] for _ in range(self.region_count)]
        self.transform_robot_y = [[] for _ in range(self.region_count)]
        self.transform_robot_z = [[] for _ in range(self.region_count)]
        self.transform_aurora_x = [[] for _ in range(self.region_count)]
        self.transform_aurora_y = [[] for _ in range(self.region_count)]
        self.transform_aurora_z = [[] for _ in range(self.region_count)]
        
        # 領域の境界値を計算
        self.x_boundaries = np.linspace(200, 300, divisions_per_axis + 1)
        self.y_boundaries = np.linspace(-50, 50, divisions_per_axis + 1)
        self.z_boundaries = np.linspace(80, 180, divisions_per_axis + 1)

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

    def calculate_errors(self, x, y, z, aurora_x, aurora_y, aurora_z, R_matrices, t_vectors):
        """座標変換と誤差計算"""
        d = []
        sample_quat = np.array([0.0, 0.0, 0.0, 1.0])
        
        for i in range(len(x)):
            xyz = np.array([x[i], y[i], z[i]])
            aurora_xyz = np.array([aurora_x[i], aurora_y[i], aurora_z[i]])
            
            region = self.get_region_number(x[i], y[i], z[i])
            
            if R_matrices[region] is not None:
                robot_pos, trash_euler, trash_quat = transform_point_and_orientation(
                    aurora_xyz,
                    sample_quat,
                    t_vectors[region],
                    R_matrices[region]
                )
                error = np.sqrt(
                    (xyz[0]-robot_pos[0])**2 +
                    (xyz[1]-robot_pos[1])**2 +
                    (xyz[2]-robot_pos[2])**2
                )
                d.append(error)
            else:
                d.append(float('nan'))
        
        return d

def visualize_results(x, y, z, d):
    """結果を3Dプロットで可視化"""
    # 平均を計算
    d_filtered = [val for val in d if not np.isnan(val)]
    ave = sum(d_filtered) / len(d_filtered)
    max_val = np.nanmax(d)
    min_val = np.nanmin(d)

    # カラーバー調整用の座標をプロット
    x = list(x) + [400, 100]
    y = list(y) + [5, 5]
    z = list(z) + [135, 135]
    d = list(d) + [0, 5]

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    map = ax.scatter(x, y, z, s=20, c=d, cmap=plt.cm.jet)
    cbar = plt.colorbar(map, ticks=mticker.LinearLocator(numticks=5))

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.title('ave:' + str(np.round(ave,3)) + '[mm]\n' + 
              str(np.round(min_val,3)) + '[mm] < diff < ' + 
              str(np.round(max_val,3)) + '[mm]')

    # auroraの位置を表示
    x = 400
    y = np.linspace(-60, 100, 11)
    z = np.linspace(40, 200, 11)
    Y, Z = np.meshgrid(y, z)
    X = np.array([x] * Y.shape[0])
    ax.plot_surface(X, Y, Z, alpha=0.3)

    # ロボットの位置を表示
    x = 100
    y = np.linspace(-60, 100, 11)
    z = np.linspace(40, 200, 11)
    Y, Z = np.meshgrid(y, z)
    X = np.array([x] * Y.shape[0])
    ax.plot_surface(X, Y, Z, alpha=0.3)

    plt.show()

def main():
    # CSVファイルの読み込み
    targetData = np.loadtxt('robot&aurora/20250214/transform_data.csv', skiprows=1, delimiter=',')
    
    # 分割数の設定（2=8領域、3=27領域、4=64領域）
    divisions = 3  # ここで分割数を変更できます
    
    # AdaptiveTransformクラスの初期化
    transformer = AdaptiveTransform(divisions)
    
    # データの処理
    x, y, z, aurora_x, aurora_y, aurora_z = transformer.process_data(targetData)
    
    # 変換行列の計算
    R_matrices, t_vectors = transformer.calculate_transformations()
    
    # 誤差計算
    d = transformer.calculate_errors(x, y, z, aurora_x, aurora_y, aurora_z, R_matrices, t_vectors)
    
    # 結果の可視化
    visualize_results(x, y, z, d)

if __name__ == "__main__":
    main()