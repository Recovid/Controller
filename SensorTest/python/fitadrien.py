import matplotlib
import matplotlib.pyplot as plt
import sys
import numpy as np

def integr(x,p):
    out = np.zeros(len(x))
    for i in range(1, len(x)):
        out[i]=out[i-1]+(x[i]-x[i-1])*p
    return out

pfu = 0
cfu = 0
P = 0
vol_b = 0
vol_b_p = 0
raw_dt_ms = 20.0
i = 0

def fit(flow):
    global P

    temp_deb = 0
    fact_erreur = 0

    if flow < 0:
        fact_erreur       = 0.0037 * P*P - 0.5124 * P + 16.376;
        temp_deb = flow * 0.88
    else:
        fact_erreur = -0.0143 * P + 1.696;
        temp_deb = flow * 0.87;
    flow_corr = temp_deb + flow * raw_dt_ms/1000.0 * fact_erreur

    return flow_corr



def main(argv):

    name=argv[1]
    global P
    val = np.loadtxt(name)
    x = val[:,0]
    paw = val[:,1]
    flow = val[:,4]
    vol = np.zeros_like(flow)
    flow_corr = np.zeros_like(flow)
    flow_tsi = np.zeros_like(flow)
    vol_corr = np.zeros_like(flow)
    vol_tsi = np.zeros_like(flow)
    P = max(paw)
    for i in range(len(vol)):
        flow_corr[i]=fit(flow[i])
        vol_corr[i]=vol_corr[i-1]+flow_corr[i]*0.020 if i>0 else 0
        #vol_corr[i]=flow_corr[i]/60.0*0.0020 if i>0 else 0
    #fig, ax = plt.subplots(1, 1)
    fig, axs = plt.subplots(2, 1)
    
    for i in range(len(vol)):
        flow_tsi[i] = val[i,2] if flow[i] > 0 else - val[i,2]
    
    for i in range(1, len(vol)):
        vol[i] = vol[i-1] + flow[i]*0.02

    for i in range(1, len(vol)):
        vol_tsi[i] = vol_tsi[i-1] + flow_tsi[i]*0.02

    axs[0].plot(x,flow )
    axs[0].plot(x,flow_corr )
    axs[0].plot(x, flow_tsi )
    axs[0].grid(True)
    axs[0].legend(['Flow Reco', 'Flow Corr', 'Flow TSI'])
    
    axs[1].plot(x,vol )
    axs[1].plot(x,vol_corr )
    axs[1].plot(x,vol_tsi )
    axs[1].grid(True)
    axs[1].legend(['Vol Reco', 'Vol Corr', 'Vol TSI'])
    #freco = ax.plot(x,flow )
    #freco = ax.plot(x,val[:,2] )
    #vcor = ax.plot(x,flow_corr )
    #vcor = ax.plot(x,val[:,3] )
    #ax.set(xlabel='time (ms)', ylabel='slm', title='Flow Sensors')
    #ax.grid()
    plt.savefig('fig')
    plt.show()
    out = np.column_stack((x, flow, flow_corr, flow_tsi, vol, vol_corr, vol_tsi))
    np.savetxt('fit_'+name, out)

    #out = np.column_stack((x_tsi, pawi_reco, paw_tsi, vol_tsi, dpi_reco, voli_reco))



if __name__ == "__main__":
    main(sys.argv)
