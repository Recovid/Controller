
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

def integr(x,p):
    out = np.zeros(len(x))
    for i in range(1, len(x)):
        out[i]=out[i-1]+(x[i]-x[i-1])*p
    return out

def main(argv):

    name=argv[1]

    recof = open(name+'recovid.txt', 'r+')
    line=recof.readline()
    while "START" not in line:
        line=recof.readline()
    starttime=None
    starttime=int(line.split()[1])
    timesum=0

    dp_reco=[]
    dp_reco_time=[]
    paw_reco=[]
    paw_reco_time=[]

    Psave=True

    for line in recof:
        if starttime is None:
            if "START" in line:
                starttime=int(line.split()[1])
            else:
                continue
        if "START" in line:
            starttime=int(line.split()[1])
        elif "P" in line:
            Psave=True
            timesum=starttime
        elif "Q" in line:
            Psave=False
            timesum=starttime
        else:
            vals= line.split(' ')
            if(len(vals)==2):
                timesum=timesum+int(vals[1])/1000.0
                if Psave:
                    paw_reco.append(int(vals[0]))
                    paw_reco_time.append(timesum)
                else:
                    dp_reco.append(int(vals[0])/20)
                    dp_reco_time.append(timesum)

    #recovid = np.loadtxt(name+'recovid.txt')
    tsi = np.loadtxt(name+'tsi.txt')
    x_tsi = tsi[:,0]
    y_tsi = tsi[:,1]
    #x_reco = recovid[:,0]
    #dp_reco =recovid[:,1]
    #paw_reco =recovid[:,2]
    #vol_reco =recovid[:,3]
    dpi_reco = np.interp(x_tsi,dp_reco_time,dp_reco)
    #voli_reco = np.interp(x_tsi,x_reco,vol_reco)
    pawi_reco = np.interp(x_tsi,paw_reco_time,paw_reco)
    #vol_tsi=integr(y_tsi,20)
    out = np.column_stack((x_tsi, pawi_reco, y_tsi, dpi_reco ))
    np.savetxt(name+'interp.txt', out)
    fig, axs = plt.subplots(2, 1)
    
    axs[0].plot(x_tsi,dpi_reco )
    axs[0].plot(x_tsi,y_tsi )
    axs[0].grid(True)
    axs[0].legend(['Flow Reco', 'Flow TSI'])
    
    axs[1].plot(x_tsi,pawi_reco )
    axs[1].grid(True)
    axs[1].legend(['Paw Reco'])
    #freco = ax.plot(x,flow )
    #freco = ax.plot(x,val[:,2] )
    #vcor = ax.plot(x,flow_corr )
    #vcor = ax.plot(x,val[:,3] )
    #ax.set(xlabel='time (ms)', ylabel='slm', title='Flow Sensors')
    #ax.grid()
    plt.savefig('fig')
    #plt.show()


if __name__ == "__main__":
    main(sys.argv)
