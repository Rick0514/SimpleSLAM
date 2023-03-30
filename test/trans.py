import tf.transformations as tr

rpy = [0.1, 0.2, 0.3]
q = tr.quaternion_from_euler(*rpy, axes='rxyz')
print(q)
