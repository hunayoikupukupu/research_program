import matplotlib.pyplot as mplot
import numpy as np

targetData=np.loadtxt('robot&aurora/20240130/test.csv',skiprows=1,delimiter=',')
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
