import matplotlib.pyplot as mplot
import numpy as np
from matplotlib.animation import FuncAnimation

targetData=np.loadtxt('robot&aurora/20240130/data_between.csv',skiprows=1,delimiter=',')
x=[]
y=[]
z=[]
d=[]


#csvファイルから要素を抽出
for xyzd in targetData:
    x.append(xyzd[0])
    y.append(xyzd[1])
    z.append(xyzd[2])
    d.append(xyzd[6]**2.4)

fig=mplot.figure()
ax=fig.add_subplot(projection='3d')
ax.scatter(x,y,z,s=d)
ax.scatter(115.6,-323.1,122.6)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

#auroraの位置を表示
x = -80
y = np.linspace(-400, -240, 11)
z = np.linspace(40, 200, 11)
Y, Z = np.meshgrid(y, z)
X = np.array([x] * Y.shape[0])
ax.plot_surface(X, Y, Z, alpha=0.3)

#ロボットの位置を表示
y = -220
x = np.linspace(-60, 20, 11)
z = np.linspace(40, 200, 11)
X, Z = np.meshgrid(x, z)
Y = np.array([y] * X.shape[0])
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
# ani.save("robot&aurora/20240130/data_between.gif", writer="pillow")