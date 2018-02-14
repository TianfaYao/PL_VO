# import necessary module
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

# load data from file
# you can replace this using with open
data1 = np.loadtxt("../cmake-build-debug/tum_trajectory1.txt")
data2 = np.loadtxt("../cmake-build-debug/groundtruth1.txt");
tx = data1[:, 1]
ty = data1[:, 2]
tz = data1[:, 3]

txg = data2[:, 1]
tyg = data2[:, 2]
tzg = data2[:, 3]


# new a figure and set it into 3d
fig = plt.figure()
ax = fig.gca(projection='3d')

# set figure information
ax.set_title("Trajectory")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

# draw the figure, the color is r = read
figure = ax.plot(tx, ty, tz, c='r')
figure2 = ax.plot(txg, tyg, tzg, c='b');

plt.show()