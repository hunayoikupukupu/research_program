import matplotlib.pyplot as mplot
import numpy as np
from matplotlib.animation import FuncAnimation

targetData=np.loadtxt('robot&aurora/20240228/data_all_0228.csv',skiprows=1,delimiter=',')
x=[]
y=[]
z=[]
d=[]


#csvファイルから要素を抽出
for xyzd in targetData:
    if xyzd[6] >= 1 :
        x.append(xyzd[0])
        y.append(xyzd[1])
        z.append(xyzd[2])
        d.append(xyzd[6]**2.4)
    

fig=mplot.figure()
ax=fig.add_subplot(projection='3d')

ax.scatter(x,y,z,s=d)
ax.scatter(334,3,142)
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
#ani.save("robot&aurora/20240228/diff_size_over1.gif", writer="pillow")