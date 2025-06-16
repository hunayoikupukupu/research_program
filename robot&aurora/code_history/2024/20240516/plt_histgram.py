import matplotlib.pyplot as plt
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
    d.append((np.sqrt(xyzd[3]-xyzd[6])**2 + (xyzd[4]-xyzd[7])**2 + (xyzd[5]-xyzd[8])**2))

plt.hist(d, bins=20, density=True)
plt.xlabel("difference[mm]")
plt.ylabel("ratio")
plt.show()