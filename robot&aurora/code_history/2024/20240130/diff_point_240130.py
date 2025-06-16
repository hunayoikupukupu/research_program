import matplotlib.pyplot as mplot
import numpy as np

targetData=np.loadtxt('robot&aurora/20240130/test.csv',skiprows=1,delimiter=',')
x=[]
y=[]
z=[]



for xyz in targetData:
    x.append(xyz[3])
    y.append(xyz[4])
    z.append(xyz[5])

fig=mplot.figure()
ax=fig.add_subplot(projection='3d')
ax.scatter(x,y,z)
ax.scatter(-9.84,-2.444,-214.304,s=100)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
mplot.show()