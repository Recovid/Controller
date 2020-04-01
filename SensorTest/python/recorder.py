import serial
import sys
import time
import numpy as np
import struct
import threading
import matplotlib.pyplot as plt


def readTSI(dev, bdrate, samplesNb, periodMs : int):
    ser = serial.Serial(dev, bdrate, timeout=10)
    command = "SSR" + str(periodMs).zfill(4)
    ser.write(command.encode())
    ser.write(b'\r')
    line = str(ser.readline())[2:][:-5]
    if line != 'OK':
        print("ERROR : TSI DIDN'T ACK COMMAND SET PERIOD TO", periodMs)
        return

    command = "DCFxx" + str(samplesNb).zfill(4)
    ser.write(command.encode())
    ser.write(b'\r')
    line = str(ser.readline())[2:][:-5]
    if line != 'OK':
        print("ERROR : TSI DIDN'T ACK COMMAND START MEASUREMENT")
        return
    counter = 0
    samples = []
    while True:
        line = str(ser.readline())
        if len(line) > 0:
            Fslm_Tsi = float(line[2:][:-5])
            counter += 1
            millis = int(round(time.time() * 1000))
            samples.append([millis, Fslm_Tsi])
            if counter == samplesNb:
                break
    stop_event.set()
    np_samples = np.array(samples)
    np.savetxt('tsi.txt', np_samples)
    print("Recovid finished", counter, "samples")


def read_recovid(dev, bdrate):
    ser = serial.Serial(dev, bdrate, timeout=10)
    samples = []
    count = 0
    while not stop_event.isSet():
        c = ser.read()
        while c == b'>' and not stop_event.isSet():
            millis = int(round(time.time() * 1000))
            t, = struct.unpack('H', ser.read(2))
            dp, = struct.unpack('f', ser.read(4))
            paw, = struct.unpack('f', ser.read(4))
            samples.append([millis, dp, paw])
            count += 1
            c = ser.read()
    np_samples = np.array(samples)
    np.savetxt('recovid.txt', np_samples)
    print("Recovid finished", count, "samples")


samples_cnt = 100
T = 50
stop_event = threading.Event()

def main(argv):
    # print(argv[1])
    thread_tsi = threading.Thread(target=readTSI, args=('/dev/ttyUSB0', 38400, samples_cnt, T))
    thread_covid = threading.Thread(target=read_recovid, args=('/dev/ttyUSB1', 115200))
    thread_tsi.start()
    thread_covid.start()
    thread_tsi.join()
    thread_covid.join()

    # tsi = np.loadtxt('tsi.txt')
    # recovid = np.loadtxt('recovid.txt')
    #
    # fig, ax = plt.subplots(1,1)
    #
    # ftsi = ax.plot(tsi[:,0], tsi[:,1])
    # freco = ax.plot(recovid[:, 0], recovid[:, 1])
    # plt.legend(['F TSI', 'F Recovid'])
    # ax.set(xlabel='time (ms)', ylabel='?', title='Sensors')
    # ax.grid()
    # plt.show()

    # print(tsi.shape, recovid.shape)
    print("main finished")


if __name__ == "__main__":
    main(sys.argv)

