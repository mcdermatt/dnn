import matplotlib.pyplot as plt
import numpy as np 
import matplotlib
import matplotlib.patches as mpatches

fn = "estimated_hip_location.npy"
arr = np.load(fn)
arr = arr[0:-1] #main doesn't generate an estimate for last sec of traj
mse = np.average(arr[:,0]**2 + arr[:,2]**2)
print(mse)

fig = plt.figure()
ax = fig.add_subplot()
ax.set_xlim([-60,60])
ax.set_xlabel("x error (in)")
ax.set_ylim([-60,60])
ax.set_ylabel("z error (in)")
plt.plot(arr[:,0],arr[:,2],'b.')
plt.plot(0,0,'r.')
plt.title("Sinusoidal Force  MSE = %s" %mse)

outfile = "figures/sinforce2.png"
# plt.savefig(outfile)
plt.show()
