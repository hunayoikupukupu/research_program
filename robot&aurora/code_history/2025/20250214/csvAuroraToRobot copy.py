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

targetData=np.loadtxt('robot&aurora/20250214/transform_data.csv',skiprows=1,delimiter=',')
ax1 = []
ay1 = []
az1 = []
ax2 = []
ay2 = []
az2 = []
d=[]


data_count = 0

# 座標データを格納するリストの初期化
x, y, z = [], [], []
aurora_x, aurora_y, aurora_z = [], [], []

# 領域ごとのデータを格納するリストの初期化
transform_robot_x = [[] for _ in range(8)]
transform_robot_y = [[] for _ in range(8)]
transform_robot_z = [[] for _ in range(8)]
transform_aurora_x = [[] for _ in range(8)]
transform_aurora_y = [[] for _ in range(8)]
transform_aurora_z = [[] for _ in range(8)]

data_count = 0

# データの振り分け
for xyz in targetData:
    x.append(xyz[0])
    y.append(xyz[1])
    z.append(xyz[2])
    aurora_x.append(xyz[3])
    aurora_y.append(xyz[4])
    aurora_z.append(xyz[5])

    # 領域の判定
    part_x = 1 if float(xyz[0]) >= 255 else 0
    part_y = 1 if float(xyz[1]) >= 5 else 0
    part_z = 1 if float(xyz[2]) >= 135 else 0
    
    part_number = int(part_x*1 + part_y*2 + part_z*4)

    # 領域ごとにデータを振り分け
    transform_robot_x[part_number].append(float(xyz[0]))
    transform_robot_y[part_number].append(float(xyz[1]))
    transform_robot_z[part_number].append(float(xyz[2]))
    transform_aurora_x[part_number].append(float(xyz[3]))
    transform_aurora_y[part_number].append(float(xyz[4]))
    transform_aurora_z[part_number].append(float(xyz[5]))

    data_count += 1

# 領域ごとの変換行列と並進ベクトルを計算
R_matrices = []
t_vectors = []
for i in range(8):
    if len(transform_robot_x[i]) > 0:  # データが存在する領域のみ処理
        P1 = np.column_stack((
            transform_robot_x[i],
            transform_robot_y[i],
            transform_robot_z[i]
        ))
        P2 = np.column_stack((
            transform_aurora_x[i],
            transform_aurora_y[i],
            transform_aurora_z[i]
        ))
        
        # 変換行列と並進ベクトルの計算
        R_matrix, t = findTransformation(P2, P1)
        R_matrices.append(R_matrix)
        t_vectors.append(np.array(t))
        
        print(f"Region {i} transformation:")
        print("Translation vector:\n", np.round(t, 4))
        print("Rotation matrix:\n", np.round(R_matrix, 4))
    else:
        R_matrices.append(None)
        t_vectors.append(None)

# 座標変換と誤差計算
d = []
sample_quat = np.array([0.0, 0.0, 0.0, 1.0])

for i in range(len(x)):
    xyz = np.array([x[i], y[i], z[i]])
    aurora_xyz = np.array([aurora_x[i], aurora_y[i], aurora_z[i]])
    
    # 点が属する領域を判定
    part_x = 1 if float(x[i]) >= 255 else 0
    part_y = 1 if float(y[i]) >= 5 else 0
    part_z = 1 if float(z[i]) >= 135 else 0
    part_number = int(part_x*1 + part_y*2 + part_z*4)
    
    # その領域の変換行列を使用して座標変換
    if R_matrices[part_number] is not None:
        robot_pos, trash_euler, trash_quat = transform_point_and_orientation(
            aurora_xyz,
            sample_quat,
            t_vectors[part_number],
            R_matrices[part_number]
        )
        robot_pos_x = robot_pos[0]
        robot_pos_y = robot_pos[1]
        robot_pos_z = robot_pos[2]
        
        # 誤差計算
        error = np.sqrt(
            (xyz[0]-robot_pos_x)**2 +
            (xyz[1]-robot_pos_y)**2 +
            (xyz[2]-robot_pos_z)**2
        )
        d.append(error)
    else:
        d.append(float('nan'))  # データが存在しない領域の場合

# 平均を計算
ave = sum(d) / len(d)
max = np.max(d)
min = np.min(d)

#カラーバー調整用の座標をプロット
x.append(400)
y.append(5)
z.append(135)
d.append(0)
x.append(100)
y.append(5)
z.append(135)
d.append(5)

fig=plt.figure()
ax=fig.add_subplot(projection='3d')

map = ax.scatter(x,y,z,s=20, c=d, cmap=plt.cm.jet)
cbar = plt.colorbar(map, ticks=mticker.LinearLocator(numticks=5))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.title('ave:' + str(np.round(ave,3)) + '[mm]\n' + str(np.round(min,3)) + '[mm] < diff < ' + str(np.round(max,3)) + '[mm]')

#auroraの位置を表示
x = 400
y = np.linspace(-60, 100, 11)
z = np.linspace(40, 200, 11)
Y, Z = np.meshgrid(y, z)
X = np.array([x] * Y.shape[0])
ax.plot_surface(X, Y, Z, alpha=0.3)

#ロボットの位置を表示
x = 100
y = np.linspace(-60, 100, 11)
z = np.linspace(40, 200, 11)
Y, Z = np.meshgrid(y, z)
X = np.array([x] * Y.shape[0])
ax.plot_surface(X, Y, Z, alpha=0.3)

plt.show()

def plot_graph():
    fig=plt.figure()
    ax=fig.add_subplot(projection='3d')

# 引数を受け取って図を回転させる関数を準備
def plt_graph3d(angle):
    ax.view_init(azim=angle)


# アニメーションを作成
ani = FuncAnimation(
    fig,
    func=plt_graph3d,
    frames=360,
    init_func=plot_graph,
    interval=100
)

# 作成したアニメーションをGIFで書き出す
# ani.save("robot&aurora/gif/p0_right_down_back.gif", writer="pillow")