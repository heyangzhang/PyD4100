from src.D_Client import D4100Client
import matplotlib.pyplot as plt

def loadImage(image_path, d_instance, devnum):
    imarray = plt.imread(image_path)
    imarray = imarray[:,:,0]
    imlist = imarray.flatten().tolist()
    d_instance._set_image(devnum, imlist)
