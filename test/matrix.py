import numpy as np


def check_rot():

    # rot = [4.85773, 0.148789, 2.17521, 0.149762, 0.0109719, -0.00465921, 2.15651, 0.0311671, 20.3052]
    rot = [1.00781,  0.000475847 , 0.0021845 , 0.00163782 ,    0.974721 ,  -0.0031621, -9.77302e-05 ,   0.0030001,      1.01525]
    rot = np.array(rot).reshape((3, 3))

    print(rot)
    print(np.matmul(rot.transpose(), rot))
    print(f"det(rot) = {np.linalg.det(rot)}")



if __name__ == "__main__":

    check_rot()