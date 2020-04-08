import matplotlib
import sys
import matplotlib.pyplot as plt
import numpy as np


def main(argv):
    # tsi = np.loadtxt('tsi.txt')
    recovid = np.loadtxt('recovid.txt')

    fig, ax = plt.subplots(1, 1)

    # ftsi = ax.plot(tsi[:, 0], tsi[:, 1])
    x = range(len(recovid))
    freco = ax.plot(x, recovid[:, 1])
    plt.legend(['Flow TSI', 'Flow Recovid SDP610 + pneumotach hamilton'])
    ax.set(xlabel='time (ms)', ylabel='slm', title='Flow Sensors')
    ax.grid()
    plt.savefig('fig')
    plt.show()



if __name__ == "__main__":
    main(sys.argv)