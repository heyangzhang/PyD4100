from src.D_Client import D4100Client
import matplotlib.pyplot as plt
import ctypes
import numpy as np

def loadImage(image_path, d_instance, devnum):
    imarray = plt.imread(image_path)
    imarray = imarray[:,:,0]
    imlist = imarray.flatten().tolist()
    #testLoad(devnum,imlist)
    d_instance._set_image(devnum, imlist)

def testLoad(devnum, imlist):
    recon = []
    for i in range(1080):
        data = imlist[i*1920:(i+1)*1920]
        recon.append(data)
        b_data = (ctypes.c_ubyte * len(data))(*data)
    reconnp = np.array(recon)
    plt.imshow(reconnp)
    plt.show()
