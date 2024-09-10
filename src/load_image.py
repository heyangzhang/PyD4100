from src.D_Client import D4100Client
import matplotlib.pyplot as plt
import ctypes
import numpy as np

def loadImage(image_path, d_instance, devnum, num_ims= 0):
    imarray = plt.imread(image_path)
    if len(imarray.shape) == 3:
        imarray = imarray[:,:,0]
    padded = 0*np.ones([1080,2048])
    padded[:,64:-64]=imarray
    plt.imshow(padded)
    image = padded.flatten().tolist()
    d_instance.set_image(devnum, image, num_ims)

def loadRaw(imarray, d_instance, devnum, num_ims = 0):
    if isinstance(imarray, list):
        image = imarray
    else:
        image = imarray.flatten().tolist()
    d_instance.set_image(devnum, image, num_ims)

def initImageList(image_path,d_instance):
    imlist = []
    for file in image_path:
        imarray = plt.imread(file)
        if len(imarray.shape) == 3:
            imarray = imarray[:,:,0]
        padded = 0*np.ones([1080,2048])
        padded[:,64:-64]=imarray
        imlist0 = padded.flatten().tolist()
        imlist.extend(imlist0)
    return imlist

def loadGray(imlist, d_instance, devnum):
    #d_instance.init_device(devnum)
    #testLoad(devnum,imlist)
    d_instance.set_gray(devnum, imlist)

def loadHalf(image_path, d_instance, devnum, num_ims= 0):
    imarray = plt.imread(image_path)
    if len(imarray.shape) == 3:
        imarray = imarray[:,:,0]
    #imarray[:,0:960] = np.zeros([1080,960])
    padded = 0*np.ones([1080,2048])
    padded[:,64:-64]=imarray
    padded = padded[:,:1024]
    plt.imshow(padded)
    image = padded.flatten().tolist()
    #image = imarray.flatten().tolist()
    d_instance.set_image(devnum, image, num_ims)


'''
def loadPadded(image_path, d_instance, devnum):
    imarray = plt.imread(image_path)
    if len(imarray.shape) == 3:
        imarray = imarray[:,:,0]
    A = imarray[::2,:960]
    B = imarray[1::2,:960]
    C = imarray[::2,960:]
    D = imarray[1::2,960:]
    concatim = np.concatenate((A,B,C,D),axis=1)
    image = concatim.flatten().tolist()
    d_instance.set_image(devnum, image)

def loadImage2(image_path, d_instance, devnum):
    imarray = plt.imread(image_path)
    if len(imarray.shape) == 3:
        imarray = imarray[:,:,0]
    imlist = imarray.flatten().tolist()
    #testLoad(devnum,imlist)
    d_instance._set_image(devnum, imlist)

def initPadImageList(image_path):
    imlist = []
    for file in image_path:
        imarray = plt.imread(file)
        if len(imarray.shape) == 3:
            imarray = imarray[:,:,0]
        imlist0 = imarray.flatten().tolist()
        imlist.append(imlist0)
    return imlist
'''