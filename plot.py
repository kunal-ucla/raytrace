from numpy import *
from matplotlib.pyplot import *
from scipy import *
from pylab import *
from mpl_toolkits.mplot3d import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from math import *
import sys

with open(sys.argv[1]) as f:
    data = f.read()

data = data.split('\n')

x = []
y = []
z = []
n = []
all_planes = []
receiver = []
transmitter = []
pdp_time = []
pdp_field = []

for row in data:
	if(row.split(" ")[0] == "planes"):
		x_plane = []
		y_plane = []
		z_plane = []
		plane = []
		for i in range(2, 26, 2):
			if row.split(" ")[i-1] == "0":
				x_plane.append(row.split(" ")[i])
			if row.split(" ")[i-1] == "1":
				y_plane.append(row.split(" ")[i])
			if row.split(" ")[i-1] == "2":
				z_plane.append(row.split(" ")[i])
		x_plane = [float(g) for g in x_plane]
		y_plane = [float(g) for g in y_plane]
		z_plane = [float(g) for g in z_plane]
		plane.append(x_plane)
		plane.append(y_plane)
		plane.append(z_plane)
		all_planes.append(plane)
	elif row.split(" ")[0] == "Receiver":
		receiver = row.split(" ")[1:]
		receiver = [float(g) for g in receiver]
	elif row.split(" ")[0] == "Transmitter":
		transmitter = row.split(" ")[1:]
		transmitter = [float(g) for g in transmitter]
	elif row.split(" ")[0] == "Time":
		pdp_time.append(1e9 * float(row.split(" ")[1]))
		pdp_field.append(10*log(float(row.split(" ")[2]),10))
		continue#do nothing
	elif row.split(" ")[0] == "":
		continue#do nothing
	elif row.split(" ")[0] == "#":
		print row
		continue#comments
	else:
		x.append(row.split(" ")[0])
		y.append(row.split(" ")[1])
		z.append(row.split(" ")[2])
		n.append(row.split(" ")[3])

x = [float(g) for g in x]
y = [float(g) for g in y]
z = [float(g) for g in z]
n = [float(g) for g in n]

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
#savefig('figure_2.png')

show()