import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Load data
log_dir = '../../test/data/log/'

fn = 'tLI'

hr = np.loadtxt(log_dir + '{}.txt'.format(fn))

n = hr.shape[0]
nn = np.arange(0, n)
x = hr[:, 0]
y = hr[:, 1]
z = hr[:, 2]

labels = ['x', 'y', 'z']

# call time
total_time = 58000
gap = total_time / 50
once = n / gap
if once < 1e-1:
    once = 1

print(once)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
line, = plt.plot([], [], "r-", animated=True)

line1, = ax.plot(nn, x, label=labels[0])
line2, = ax.plot(nn, y, label=labels[1])
line3, = ax.plot(nn, z, label=labels[2])

ax.set_title(fn)
ax.grid(True)
ax.legend()

def init():
    return line1, line2, line3

def update(num):
    num = int(num)
    line1.set_data(nn[:num], x[:num])
    line2.set_data(nn[:num], y[:num])
    line3.set_data(nn[:num], z[:num])
    return line1, line2, line3

ani = FuncAnimation(fig
                   ,update
                   ,init_func=init
                   ,frames=np.arange(1, n, once)
                   ,interval=1
                   ,blit=True
                   ,repeat=False
                   )

# plt.show()
ani.save(log_dir + "{}.mp4".format(fn), fps=20, writer="ffmpeg")


