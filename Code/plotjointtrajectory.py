# importing mplot3d toolkits, numpy and matplotlib
import numpy as np
import matplotlib.pyplot as plt
import pickle

trajfile = open("joint_trajectories.noobs", "rb")
traj = pickle.load(trajfile)
trajfile.close()

traj1 = [x[0] for x in traj]
traj2 = [x[1] for x in traj]
traj3 = [x[2] for x in traj]
traj4 = [x[3] for x in traj]
traj5 = [x[4] for x in traj]
traj6 = [x[5] for x in traj]
traj7 = [x[6] for x in traj]

time = list(range(len(traj)))

#joint1
plt.plot(time, traj1,  label = "Joint 1")
#joint2
plt.plot(time, traj2,  label = "Joint 2")
#joint3
plt.plot(time, traj3,  label = "Joint 3")
#joint4
plt.plot(time, traj4,  label = "Joint 4")
#joint5
plt.plot(time, traj5,  label = "Joint 5")
#joint6
plt.plot(time, traj6,  label = "Joint 6")
#joint7
plt.plot(time, traj7,  label = "Joint 7" , alpha=0.2)
  
# naming the x axis
plt.xlabel('T')
# naming the y axis
plt.ylabel('θ')
# giving a title to my graph
plt.title('Robot joint')
  
# show a legend on the plot
plt.legend()
  
# function to show the plot
plt.show()