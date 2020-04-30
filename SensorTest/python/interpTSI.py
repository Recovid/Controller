
import sys
import numpy as np

def integr(x,p):
    out = np.zeros(len(x))
    for i in range(1, len(x)):
        out[i]=out[i-1]+(x[i]-x[i-1])*p
    return out

def main(argv):

    name=argv[1]

    tsi = np.loadtxt(name+'tsi.txt')
    recovid = np.loadtxt(name+'recovid.txt')
    x_tsi = tsi[:,0]
    y_tsi = tsi[:,1]
    x_reco = recovid[:,0]
    dp_reco =recovid[:,1]
    paw_reco =recovid[:,2]
    vol_reco =recovid[:,3]
    dpi_reco = np.interp(x_tsi,x_reco,dp_reco)
    voli_reco = np.interp(x_tsi,x_reco,vol_reco)
    pawi_reco = np.interp(x_tsi,x_reco,paw_reco)
    vol_tsi=integr(y_tsi,20)
    out = np.column_stack((x_tsi, pawi_reco, y_tsi, vol_tsi, dpi_reco, voli_reco))
    np.savetxt(name+'interp.txt', out)


if __name__ == "__main__":
    main(sys.argv)
