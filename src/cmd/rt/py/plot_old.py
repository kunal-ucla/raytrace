from numpy import *
from matplotlib.pyplot import *
from scipy import *
from pylab import *
from mpl_toolkits.mplot3d import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from math import *
import sys
import json

with open(sys.argv[1]) as data_file:    
    data = json.load(data_file)

x = []
y = []
z = []
n = []
all_planes = []
receiver = []
transmitter = []
pdp_time = []
pdp_field = []

for planes in data["Planes"]:
	plane=[]
	plane.append(list(zip(*planes))[0])
	plane.append(list(zip(*planes))[1])
	plane.append(list(zip(*planes))[2])
	all_planes.append(plane)

receiver=data["Receiver"]
transmitter=data["Transmitter"]
pdp_time=list(zip(*data["Time"]))[0]
pdp_time=[i * 1e9 for i in pdp_time]
pdp_field=list(zip(*data["Time"]))[1]
pdp_field=[10*log(i,10) for i in pdp_field]
x=list(zip(*data["Points"]))[0]
y=list(zip(*data["Points"]))[1]
z=list(zip(*data["Points"]))[2]
n=list(zip(*data["Points"]))[3]

fig = figure(1)
ax = Axes3D( fig )
ax.set_xlim([-6.5,6.5])
ax.set_ylim([-4.3,4.3])
ax.set_zlim([-1.5,1.5])
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_xticks([])
ax.set_yticks([])
ax.set_zticks([])
i = 0
for j in range(0,len(n)):
	k = n[i]
	xx = []
	yy = []
	zz = []
	while( n[i] == k ):
		xx.append(x[i])
		yy.append(y[i])
		zz.append(z[i])
		i = i + 1
		if i >= len(n):
			break
	ax.plot(xx,yy,zs=zz,alpha=0.6,color='r')
	if i >= len(n):
		break

xx1, yy1 = np.meshgrid(range(10), range(10))

color = (0. , 1. , 0. , 0.07)
color2 = (0. , 0. , 1. , 0.27)
color3 = (1. , 0. , 0. , 0.27)
for i in range(0,len(all_planes)):
	verts = [zip(all_planes[i][0], all_planes[i][1], all_planes[i][2])]
	v = art3d.Poly3DCollection(verts)
	v.set_facecolor(color)
	v.set_edgecolor(color)
	ax.add_collection3d(v)

phi = np.linspace(0, 2 * np.pi, 100)
theta = np.linspace(0, np.pi, 100)
xm = receiver[3] * np.outer(np.cos(phi), np.sin(theta)) + receiver[0]
ym = receiver[3] * np.outer(np.sin(phi), np.sin(theta)) + receiver[1]
zm = receiver[3] * np.outer(np.ones(np.size(phi)), np.cos(theta)) + receiver[2]
ax.plot_surface(xm, ym, zm, color=color2, linewidth=0)

xm = 0.1 * np.outer(np.cos(phi), np.sin(theta)) + transmitter[0]
ym = 0.1 * np.outer(np.sin(phi), np.sin(theta)) + transmitter[1]
zm = 0.1 * np.outer(np.ones(np.size(phi)), np.cos(theta)) + transmitter[2]
ax.plot_surface(xm, ym, zm, color=color3, linewidth=0)

#savefig('figure_1.png')
figure(2)
scatter(pdp_time,pdp_field)
title(r"Power delay profile")
xlabel(r"Time taken to reach in nanoseconds",size=13)
ylabel(r"Power of the rays at the receiver",size=13)
#savefig('figure_2.png')

show()
