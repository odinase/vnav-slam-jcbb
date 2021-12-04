import numpy as np
import matplotlib.pyplot as plt


def plot_log():
    path = np.loadtxt("./logs/path.txt")
    positions = path[:,:2]
    yaws = path[:,-1]

    lmks = np.loadtxt("./logs/landmarks.txt")

    fig, ax = plt.subplots()
    ax.plot(*positions.T, 'b--', lw=1.2)
    ax.plot(*positions[-1], 'b', marker=(3, 0, yaws[-1]-np.pi/2), ms=3)
    ax.plot(*lmks.T, 'r^', ms=3)

    plt.show()


if __name__ == "__main__":
    plot_log()