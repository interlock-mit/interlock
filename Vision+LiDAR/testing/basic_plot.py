# Only works for Python 3.6 or above
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import time

def plot_two(points1, points2, num, delta, title='Ground Segmentation'):
    fig = plt.figure()
    plt.axis('equal')
    plt.title(title)
    ax = fig.add_subplot(111, projection='3d')
    alpha = 0.5
    ax.scatter([p[0] for p in points1], 
               [p[1] for p in points1],
               [p[2] for p in points1], color=[1., alpha, alpha])
    ax.scatter([p[0] for p in points2], 
               [p[1] for p in points2],
               [p[2] for p in points2], color=[alpha, alpha, 1.])
    plt.savefig(f'pics/ground_segmentation{num}trials{delta}delta{time.time()}.png')
    plt.show()