import matplotlib.pyplot as mplot
import numpy as np
from matplotlib.animation import FuncAnimation

targetData=np.loadtxt('robot&aurora/20240516/data_transformation.csv',skiprows=1,delimiter=',')
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

ave = sum(d) / len(d)
print("diff_ave = ")
print(ave)

fig=mplot.figure()
ax=fig.add_subplot(projection='3d')

ax.scatter(x,y,z,s=10, c=d, cmap=mplot.cm.jet)
# bar=mplot.colorbar()
# ax=bar.add_lines

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

#auroraの位置を表示
x = 530
y = np.linspace(-80, 100, 11)
z = np.linspace(40, 200, 11)
Y, Z = np.meshgrid(y, z)
X = np.array([x] * Y.shape[0])
ax.plot_surface(X, Y, Z, alpha=0.3)

#ロボットの位置を表示
x = 0
y = np.linspace(-80, 100, 11)
z = np.linspace(40, 200, 11)
Y, Z = np.meshgrid(y, z)
X = np.array([x] * Y.shape[0])
ax.plot_surface(X, Y, Z, alpha=0.3)

mplot.show()

def plot_graph():
    fig=mplot.figure()
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
# ani.save("robot&aurora/20240228/diff_size_all.gif", writer="pillow")