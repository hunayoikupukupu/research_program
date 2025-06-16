import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.ticker as mticker

targetData=np.loadtxt('robot&aurora/20240718/p0_right_down_back.csv',skiprows=1,delimiter=',')
x=[]
y=[]
z=[]
ax1 = []
ay1 = []
az1 = []
ax2 = []
ay2 = []
az2 = []
d=[]


#csvファイルから要素を抽出
for xyzd in targetData:
    x.append(xyzd[0])
    y.append(xyzd[1])
    z.append(xyzd[2])
    d.append(np.sqrt((xyzd[3]-xyzd[6])**2 + (xyzd[4]-xyzd[7])**2 + (xyzd[5]-xyzd[8])**2))

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

# plt.show()

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
ani.save("robot&aurora/gif/p0_right_down_back.gif", writer="pillow")