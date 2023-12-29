import numpy as np
import matplotlib.pyplot as plt
# Load data
log_dir = '../../test/data/log/'

hr = np.loadtxt(log_dir + 'hr.txt')
RLI = np.loadtxt(log_dir + 'RLI.txt')
RWI = np.loadtxt(log_dir + 'RWI.txt')
tLI = np.loadtxt(log_dir + 'tLI.txt')
bgba = np.loadtxt(log_dir + 'bgba.txt')

plt.figure(1)

tt = hr[:, 0] - hr[0, 0]
plt.plot(tt, hr[:, 1], linewidth=2)
plt.plot(tt, hr[:, 2], linewidth=2)
plt.grid()
plt.legend(['$h$', '$r$'])
plt.xlabel('$t/s$')
plt.ylabel('$m$')
plt.title('$h$ vs. $r$')

plt.figure(2)
tt = RLI[:, 0] - RLI[0, 0]
plt.plot(tt, RLI[:, 1], linewidth=2)
plt.plot(tt, RLI[:, 2], linewidth=2)
plt.plot(tt, RLI[:, 3], linewidth=2)
plt.grid()
plt.legend(['roll', 'pitch', 'yaw'])
plt.xlabel('$t/s$')
plt.ylabel('deg')
plt.title('$^IR_L$')

plt.figure(3)
tt = RWI[:, 0] - RWI[0, 0]
plt.plot(tt, RWI[:, 1], linewidth=2)
plt.plot(tt, RWI[:, 2], linewidth=2)
plt.plot(tt, RWI[:, 3], linewidth=2)
plt.grid()
plt.legend(['roll', 'pitch', 'yaw'])
plt.xlabel('$t/s$')
plt.ylabel('deg')
plt.title('$^IR_W$')

plt.figure(4)
tt = tLI[:, 0] - tLI[0, 0]
plt.plot(tt, tLI[:, 1], linewidth=2)
plt.plot(tt, tLI[:, 2], linewidth=2)
plt.plot(tt, tLI[:, 3], linewidth=2)
plt.grid()
plt.legend(['x', 'y', 'z'])
plt.xlabel('$t/s$')
plt.ylabel('$m$')
plt.title('$^It_L$')

plt.figure(5)
plt.subplot(1, 2, 1)
tt = bgba[:, 0] - bgba[0, 0]
plt.plot(tt, bgba[:, 1], linewidth=2)
plt.plot(tt, bgba[:, 2], linewidth=2)
plt.plot(tt, bgba[:, 3], linewidth=2)
plt.grid()
plt.legend(['$bg_x$', '$bg_y$', '$bg_z$'])
plt.xlabel('$t/s$')
plt.ylabel('$rad/s$')
plt.title('bg')

plt.subplot(1, 2, 2)
plt.plot(tt, bgba[:, 4], linewidth=2)
plt.plot(tt, bgba[:, 5], linewidth=2)
plt.plot(tt, bgba[:, 6], linewidth=2)
plt.grid()
plt.legend(['$ba_x$', '$ba_y$', '$ba_z$'])
plt.xlabel('$t/s$')
plt.ylabel('$m/s^2$')
plt.title('ba')


plt.show()