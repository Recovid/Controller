import serial
import sys
import time
import numpy as np
import struct
import threading
import matplotlib.pyplot as plt


startTime = int(round(time.time() * 1000))
tsi_sample = []
dp_sample = []
paw_sample = []

def readTSI(dev, bdrate, samplesNb, periodMs : int, name):
    global startTime
    global tsi_sample
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
    tsi_samples = np.array(samples)
    np.savetxt(name+'_tsi.txt', tsi_samples)
    print("TSI finished", backnb, "samples")


def read_recovid(dev, bdrate, name):
    global startTime
    global dp_sample
    global paw_sample
    ser = serial.Serial(dev, bdrate, timeout=10)
    reco_start = None
    timesum=0
    Psave=True
    
    while True:
        if stop_event.isSet():
            break
        line = ser.readline()
        line=line.decode('utf-8').rstrip('\n')
        if reco_start is None:
            if "START" == line:
                print("INIT")
                millis = int(round(time.time() * 1000) - startTime)
                reco_start=millis
            else:
                continue
        if "START" in line:
            millis = int(round(time.time() * 1000) - startTime)
            reco_start=millis
            print(reco_start)
        elif "P" in line:
            print("P")
            Psave=True
            timesum=reco_start
        elif "Q" in line:
            print("Q")
            Psave=False
            timesum=reco_start
        else:
            vals = line.split(' ')
            if(len(vals)==2):
                timesum=timesum+int(vals[1])/1000.0
                if Psave:
                    paw_sample.append([timesum,int(vals[0])])
                else:
                    #print(int(vals[0]))
                    dp_sample.append([timesum,int(vals[0])])
    dp_sample=np.array(dp_sample)
    paw_sample=np.array(paw_sample)
    print(dp_sample)
    print(paw_sample)
    f = open(name+'_recovid.txt', 'a+')
    np.savetxt(f,np.array(dp_sample))
    np.savetxt(f,np.array(paw_sample))
    f.close()
    print("Recovid finished")


samples_cnt = 800
T = 20
samples_cnt = int(1000/25*60)
stop_event = threading.Event()

def main(argv):
    global dp_sample
    global paw_sample
    global tsi_sample
    
    samples_cnt = int(800/25*int(argv[1]))
    name=argv[2]
    thread_tsi = threading.Thread(target=readTSI, args=('/dev/ttyUSB0', 38400, samples_cnt, T, name))
    thread_covid = threading.Thread(target=read_recovid, args=('/dev/serial0', 115200, name))
    thread_tsi.start()
    thread_covid.start()
    thread_tsi.join()
    thread_covid.join()

    tsi = np.loadtxt(name+'_tsi.txt')
    
    time_tsi = tsi[:,0]
    dp_tsi = tsi[:,1]
    
    dpi_reco = np.interp(time_tsi,dp_sample[:,0],dp_sample[:,1])
    pawi_reco = np.interp(time_tsi,paw_sample[:,0],paw_sample[:,1])
    
    out = np.column_stack((time_tsi, pawi_reco, dp_tsi, dpi_reco ))
    np.savetxt(name+'_interp.txt', out)

    fig, axs = plt.subplots(2, 1)
    
    axs[0].plot(time_tsi,dpi_reco*-1/105000 )
    axs[0].plot(time_tsi,dp_tsi)
    axs[0].grid(True)
    axs[0].legend(['Flow Reco', 'Flow TSI'])
    
    axs[1].plot(time_tsi,pawi_reco )
    axs[1].grid(True)
    axs[1].legend(['Paw Reco'])
    plt.savefig('fig')
    plt.show()

    print("main finished")


if __name__ == "__main__":
    main(sys.argv)

