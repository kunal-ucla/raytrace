from numpy import *
from matplotlib.pyplot import *
from scipy import *
from pylab import *
from mpl_toolkits.mplot3d import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.collections import PolyCollection
from math import *
import sys
import json

with open(sys.argv[1]) as data_file:    
    data = json.load(data_file)

xs=[]
ys=[]
zs=[]

for i in data["Data"]:
	ys=[]
	for j in i:
		ys.append(j["TotalField"])
	zs.append(ys)
zs=list(reversed(zs))
j=0
for i in zs:
	zs[j]=list(reversed(i))
	j=j+1



plotx = array(zs)
im=imshow(plotx,interpolation='none',aspect=13./8.6,vmin=min(ys),vmax=max(ys))
cbar = colorbar(im) 
cbar.set_label('Filed Strength: x times the transmitted field strength',size=13)
cbar.set_clim(min(ys), 10*max(ys))
jet()
show()
