from src.D_Client import D4100Client
import matplotlib.pyplot as plt
import ctypes
import numpy as np

def loadImage(image_path, d_instance, devnum):
    imarray = plt.imread(image_path)
    if len(imarray.shape) == 3:
        imarray = imarray[:,:,0]
    #imarray[:,0:960] = np.zeros([1080,960])
    padded = 0*np.ones([1080,2048])
    padded[:,64:-64]=imarray
    plt.imshow(padded)
    image = padded.flatten().tolist()
    #image = imarray.flatten().tolist()
    d_instance.set_image(devnum, image)

def loadRaw(imarray, d_instance, devnum):
    #padded = 0*np.ones([1080,2048])
    #padded[:,64:-64]=imarray
    #image = padded.flatten().tolist()
    image = imarray.flatten().tolist()
    d_instance.set_image(devnum, image)

def loadPadded(image_path, d_instance, devnum):
    imarray = plt.imread(image_path)
    if len(imarray.shape) == 3:
        imarray = imarray[:,:,0]
    image = imarray.flatten().tolist()
    d_instance.set_image(devnum, image)

def initImageList(image_path):
    imlist = []
    for file in image_path:
        imarray = plt.imread(file)
        if len(imarray.shape) == 3:
            imarray = imarray[:,:,0]
        #padded = 0*np.ones([1080,2048])
        #padded[:,64:-64]=imarray
        imlist0 = imarray.flatten().tolist()
        imlist.append(imlist0)
    return imlist

def initPadImageList(image_path):
    imlist = []
    for file in image_path:
        imarray = plt.imread(file)
        if len(imarray.shape) == 3:
            imarray = imarray[:,:,0]
        imlist0 = imarray.flatten().tolist()
        imlist.append(imlist0)
    return imlist

def loadGray(imlist, d_instance, devnum):
    #d_instance.init_device(devnum)
    #testLoad(devnum,imlist)
    d_instance.set_gray(devnum, imlist)

def testLoad(devnum, imlist):
    recon = []
    for i in range(1080):
        data = imlist[i*1920:(i+1)*1920]
        recon.append(data)
        b_data = (ctypes.c_ubyte * len(data))(*data)
    reconnp = np.array(recon)
    plt.imshow(reconnp)
    plt.show()
