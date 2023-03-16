import numpy as np
import matplotlib.pyplot as plt


fn = './data/loam.txt'

nearest_dist = []
with open(fn, 'r') as f:
    line = f.readline()
    while len(line):
        if line.startswith('nks'):
            nearest_dist.append(float(line.strip('\n').split(' ')[1]))
    
        line = f.readline()

n = len(nearest_dist)
hist, be = np.histogram(nearest_dist, bins=0.1 * np.arange(0, 10, 1))

perc = hist / n
print(perc)
print(np.sum(perc))
print(be)