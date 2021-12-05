import numpy as np
import matplotlib.pyplot as plt


def plot_log():
    path_jcbb = np.loadtxt("./logs/path_good3.txt")

    lmks_jcbb = np.loadtxt("./logs/landmarks_good3.txt")

    path_ml = np.loadtxt("./logs/path_ml.txt")

    lmks_ml = np.loadtxt("./logs/landmarks_ml.txt")

    odoms = np.loadtxt("./logs/odom.txt")

    fig, ax = plt.subplots()
    ax.plot(*path_jcbb[:,:2].T, 'b', lw=1.2, label='Robot path JCBB')
    ax.plot(*path_jcbb[-1,:2], 'b', marker=(3, 0, path_jcbb[-1,-1]-np.pi/2), ms=3)
    ax.plot(*lmks_jcbb.T, 'r^', ms=3, label='landmarks JCBB')

    ax.plot(*path_ml[:,:2].T, 'g--', lw=1.2, label='Robot path ML')
    ax.plot(*path_ml[-1,:2], 'g', marker=(3, 0, path_ml[-1,-1]-np.pi/2), ms=3)
    ax.plot(*lmks_ml.T, 'k^', ms=3, label='landmarks ML')

    ax.plot(*odoms[:,:2].T, 'r-.', lw=1.2, label='odom')
    ax.legend()

    plt.show()


if __name__ == "__main__":
    plot_log()