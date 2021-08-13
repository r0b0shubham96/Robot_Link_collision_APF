# importing mplot3d toolkits, numpy and matplotlib
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import pickle

trajfile = open("trajectory.obs", "rb")
traj = pickle.load(trajfile)
trajfile.close()

trajfile = open("trajectory.noobs", "rb")
traj1 = pickle.load(trajfile)
trajfile.close()

fig = plt.figure()
 
# syntax for 3-D projection
ax = plt.axes(projection ='3d')

spradius = 0.18
center = [-0.4 , -0.05,  0.45]
u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
x = spradius*np.cos(u)*np.sin(v) +center[0]
y = spradius*np.sin(u)*np.sin(v) +center[1]
z = spradius*np.cos(v) +center[2]
ax.plot_wireframe(x, y, z, color="grey")

# defining all 3 axes
x = [x[0] for x in traj]
y = [x[1] for x in traj]
z = [x[2] for x in traj]
 
# plotting
ax.plot3D(x, y, z, 'red')

x = [x[0] for x in traj1]
y = [x[1] for x in traj1]
z = [x[2] for x in traj1]
 
# plotting
ax.plot3D(x, y, z, 'green')
ax.set_title("X_Traj(green) vs X_Rep_Traj(red) ")
plt.show()