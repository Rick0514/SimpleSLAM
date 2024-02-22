import numpy as np

import matplotlib
matplotlib.use("Qt4Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# Load data
log_dir = '../../test/data/log/'

hr = np.loadtxt(log_dir + 'hr.txt')
RLI = np.loadtxt(log_dir + 'RLI.txt')
RWI = np.loadtxt(log_dir + 'RWI.txt')
tLI = np.loadtxt(log_dir + 'tLI.txt')
bgba = np.loadtxt(log_dir + 'bgba.txt')

plt.figure(1)
ax = plt.gca()

tt = hr[:, 0] - hr[0, 0]
plt.plot(tt, hr[:, 1], linewidth=2)
plt.plot(tt, hr[:, 2], linewidth=2)
ax.xaxis.set_tick_params(labelsize=15)
ax.yaxis.set_tick_params(labelsize=15)
plt.legend(['$\\bf{h}$', '$\\bf{r}$'], prop={'size': 15})
plt.xlabel('time/s', fontsize=15, fontweight='bold')
plt.ylabel('length/m', fontsize=15, fontweight='bold')
plt.title('$\\bf{h}$ vs. $\\bf{r}$', fontsize=20, fontweight='bold')
plt.grid()

plt.figure(2)
ax = plt.gca()
tt = RLI[:, 0] - RLI[0, 0]
plt.plot(tt, RLI[:, 1], linewidth=2)
plt.plot(tt, RLI[:, 2], linewidth=2)
plt.plot(tt, RLI[:, 3], linewidth=2)
ax.xaxis.set_tick_params(labelsize=15)
ax.yaxis.set_tick_params(labelsize=15)
plt.grid()
plt.legend(['roll', 'pitch', 'yaw'], prop={'size': 15, 'weight':'bold'})
plt.xlabel('time/s', fontsize=15, fontweight='bold')
plt.ylabel('angle/deg', fontsize=15, fontweight='bold')
plt.title('$\\bf{^IR_L}$', fontsize=20, fontweight='bold')

plt.figure(3)
ax = plt.gca()
tt = RWI[:, 0] - RWI[0, 0]
plt.plot(tt, RWI[:, 1], linewidth=2)
plt.plot(tt, RWI[:, 2], linewidth=2)
plt.plot(tt, RWI[:, 3], linewidth=2)
ax.xaxis.set_tick_params(labelsize=15)
ax.yaxis.set_tick_params(labelsize=15)
plt.grid()
plt.legend(['roll', 'pitch', 'yaw'], prop={'size': 15, 'weight':'bold'})
plt.xlabel('time/s', fontsize=15, fontweight='bold')
plt.ylabel('angle/deg', fontsize=15, fontweight='bold')
plt.title('$\\bf{^IR_W}$', fontsize=20, fontweight='bold')

plt.figure(4)
ax = plt.gca()
tt = tLI[:, 0] - tLI[0, 0]
plt.plot(tt, tLI[:, 1], linewidth=2)
plt.plot(tt, tLI[:, 2], linewidth=2)
plt.plot(tt, tLI[:, 3], linewidth=2)
ax.xaxis.set_tick_params(labelsize=15)
ax.yaxis.set_tick_params(labelsize=15)
plt.grid()
plt.legend(['x', 'y', 'z'], prop={'size': 15, 'weight':'bold'})
plt.xlabel('time/s', fontsize=15, fontweight='bold')
plt.ylabel('length/m', fontsize=15, fontweight='bold')
plt.title('$^It_L$', fontsize=20, fontweight='bold')

plt.figure(5)
plt.subplot(1, 2, 1)
ax = plt.gca()
tt = bgba[:, 0] - bgba[0, 0]
plt.plot(tt, bgba[:, 1], linewidth=2)
plt.plot(tt, bgba[:, 2], linewidth=2)
plt.plot(tt, bgba[:, 3], linewidth=2)
plt.grid()
ax.xaxis.set_tick_params(labelsize=15)
ax.yaxis.set_tick_params(labelsize=15)
plt.legend(['$\\bf{bg_x}$', '$\\bf{bg_y}$', '$\\bf{bg_z}$'], prop={'size': 15, 'weight':'bold'})
plt.xlabel('time/s', fontsize=15, fontweight='bold')
plt.ylabel('$\\bf{rad/s}$', fontsize=15, fontweight='bold')
plt.title('bg', fontsize=20, fontweight='bold')

plt.subplot(1, 2, 2)
ax = plt.gca()
plt.plot(tt, bgba[:, 4], linewidth=2)
plt.plot(tt, bgba[:, 5], linewidth=2)
plt.plot(tt, bgba[:, 6], linewidth=2)
plt.grid()
ax.xaxis.set_tick_params(labelsize=15)
ax.yaxis.set_tick_params(labelsize=15)
plt.legend(['$\\bf{ba_x}$', '$\\bf{ba_y}$', '$\\bf{ba_z}$'], prop={'size': 15, 'weight':'bold'})
plt.xlabel('time/s', fontsize=15, fontweight='bold')
plt.ylabel('$\\bf{m/s^2}$', fontsize=15, fontweight='bold')
plt.title('ba', fontsize=20, fontweight='bold')


plt.show()