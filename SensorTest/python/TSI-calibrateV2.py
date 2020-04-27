import serial
import sys
import time
import numpy as np
import struct
import threading
import matplotlib.pyplot as plt


def readTSI(dev, bdrate, samplesNb, periodMs : int, name):
    ser = serial.Serial(dev, bdrate, timeout=10)
    backnb=samplesNb
    command = "SSR" + str(periodMs).zfill(4)
    ser.write(command.encode())
    ser.write(b'\r')
    line = str(ser.readline())[2:][:-5]
    if line != 'OK':
        print("ERROR : TSI DIDN'T ACK COMMAND SET PERIOD TO", periodMs)
        return
    sploop = samplesNb if samplesNb < 800 else 800
    command = "DCFxx" + str(sploop).zfill(4)
    print(command)
    ser.write(command.encode())
    ser.write(b'\r')
    line = str(ser.readline())[2:][:-5]
    if line != 'OK':
        print("ERROR : TSI DIDN'T ACK COMMAND START MEASUREMENT")
        return
    counter = 0
    samples = []
    startTime = int(round(time.time() * 1000))
    while True:
        line = str(ser.readline())
        if len(line) > 3:
            try:
                Fslm_Tsi = float(line[2:][:-5])
            except:
                print("ERROR")
                print(line)
                return
            counter += 1
            millis = int(round(time.time() * 1000) - startTime)
            samples.append([millis, Fslm_Tsi])
            if counter == 800 and samplesNb > 800:
                counter=0
                samplesNb = samplesNb-800
                sploop = samplesNb if samplesNb < 800 else 800
                command = "DCFxx" + str(sploop).zfill(4)
                ser.write(command.encode())
                ser.write(b'\r')
                print(command)
                line = str(ser.readline())[2:][:-5]
                if line != 'OK':
                    print("ERROR : TSI DIDN'T ACK COMMAND START MEASUREMENT")
                    return
            elif counter == samplesNb:
                stop_event.set()
                break
    np_samples = np.array(samples)
    np.savetxt(name+'_tsi.txt', np_samples)
    print("TSI finished", backnb, "samples")


def read_recovid(dev, bdrate, name):
    ser = serial.Serial(dev, bdrate, timeout=10)
    f = open(name+'_recovid.txt', 'w+')
    startTime = int(round(time.time() * 1000))
    while True:
        if stop_event.isSet():
            break
        line = ser.readline()
        line=line.decode('utf-8').rstrip('\n')
        if line=='START':
            f.flush()
            millis = int(round(time.time() * 1000) - startTime)
            print("START", millis, file=f)
        else:
            print(line, file=f)
    f.close()
    print("Recovid finished")


samples_cnt = 800
T = 20
samples_cnt = int(1000/25*60)
stop_event = threading.Event()

def main(argv):
    samples_cnt = int(800/25*int(argv[1]))
    name=argv[2]
    thread_tsi = threading.Thread(target=readTSI, args=('/dev/ttyUSB0', 38400, samples_cnt, T, name))
    thread_covid = threading.Thread(target=read_recovid, args=('/dev/serial0', 115200, name))
    thread_tsi.start()
    thread_covid.start()
    thread_tsi.join()
    thread_covid.join()

    #tsi = np.loadtxt(name+'_tsi.txt')
    #recovid = np.loadtxt(name+'_recovid.txt')

    #fig, ax = plt.subplots(1, 1)

    #ftsi = ax.plot(tsi[:,0], tsi[:,1])
    #freco = ax.plot(recovid[:, 0], recovid[:, 1])
    #plt.legend(['F TSI', 'F Recovid'])
    #ax.set(xlabel='time (ms)', ylabel='?', title='Sensors')
    #ax.grid()
    #plt.show()

    # print(tsi.shape, recovid.shape)
    print("main finished")


if __name__ == "__main__":
    main(sys.argv)

