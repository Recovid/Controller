import struct
import matplotlib.pyplot as plt
import numpy as np
import serial
import sys


# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')


def live_plotter(x_vec, y_data, lines, identifier='', pause_time=0.001):
    if lines[0] == []:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()

        fig, axs = plt.subplots(3, 1)

        axs[0].set_xlabel('time')
        axs[0].set_ylabel('Flow [slm]')
        axs[0].grid(True)
        axs[1].set_xlabel('time')
        axs[1].set_ylabel('Paw [cm H2O]')
        axs[1].grid(True)
        axs[2].set_xlabel('time')
        axs[2].set_ylabel('Volume [ml]')
        axs[2].grid(True)

        # create a variable for the line so we can later update it
        lines[0], = axs[0].plot(x_vec, y_data[0], '-', alpha=0.8)
        lines[1], = axs[1].plot(x_vec, y_data[1], '-', alpha=0.8)
        lines[2], = axs[2].plot(x_vec, y_data[2], '-', alpha=0.8)
        plt.show()

    # after the figure, axis, and line are created, we only need to update the y-data
    # adjust limits if new data goes beyond bounds
    lines[0].set_ydata(y_data[0])
    lines[1].set_ydata(y_data[1])
    lines[2].set_ydata(y_data[2])
    if np.min(y_data[0]) <= lines[0].axes.get_ylim()[0] or np.max(y_data[0]) >= lines[0].axes.get_ylim()[1]:
        lines[0].axes.set_ylim([np.min(y_data[0]) - np.std(y_data[0]), np.max(y_data[0]) + np.std(y_data[0])])
    if np.min(y_data[1]) <= lines[1].axes.get_ylim()[0] or np.max(y_data[1]) >= lines[1].axes.get_ylim()[1]:
        lines[1].axes.set_ylim([np.min(y_data[1]) - np.std(y_data[1]), np.max(y_data[1]) + np.std(y_data[1])])
    if np.min(y_data[2]) <= lines[2].axes.get_ylim()[0] or np.max(y_data[2]) >= lines[2].axes.get_ylim()[1]:
        lines[2].axes.set_ylim([np.min(y_data[2]) - np.std(y_data[2]), np.max(y_data[2]) + np.std(y_data[2])])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)

    # return line so we can update it again in the next iteration
    return lines



size=100

time = np.linspace(0, 1, size+1)[0:-1]
flows = np.zeros(size)
paws = np.zeros(size)
vols = np.zeros(size)
data = [[], [], []]
data = live_plotter(time, [flows, paws, vols], data, pause_time=0.001)


with serial.Serial('/dev/ttyUSB0', 115200, timeout=None) as ser:
    while True:
        c = ser.read()
        while c == b'>':
            # t, = struct.unpack('H', ser.read(2))
            dp, = struct.unpack('f', ser.read(4))
            paw, = struct.unpack('f', ser.read(4))
            vol, = struct.unpack('f', ser.read(4))
            # dp = dp * 1.1 * 0.51
            #time[-1]=t
            flows[-1] = dp
            paws[-1] = paw
            vols[-1] = vol

            plot_data = [flows, paws, vols]

            data = live_plotter(time, plot_data, data, pause_time=0.01)

            paws = np.append(paws[1:], 0.0)
            flows = np.append(flows[1:], 0.0)
            vols = np.append(vols[1:], 0.0)
            c = ser.read()

