from src.D_Client import D4100Client
from src.load_image import loadGray
from src.load_image import loadImage
from src.load_image import initImageList
from src.load_image import initPadImageList
from src.load_image import loadPadded
from src.load_image import loadRaw
import matplotlib.pyplot as plt
import numpy as np
import pathlib
from time import sleep


im = "lib/checker1.jpg"
im2 = "lib/line.bmp"
im3 = "lib/TI Logo.bmp"
#files_n = ["lib/narrow/gradient_0.bmp","lib/narrow/gradient_1.bmp","lib/narrow/gradient_2.bmp","lib/narrow/gradient_3.bmp","lib/narrow/gradient_4.bmp","lib/narrow/gradient_5.bmp","lib/narrow/gradient_6.bmp","lib/narrow/gradient_7.bmp"]
files_w = ["lib/wide/gradient_0.bmp","lib/wide/gradient_1.bmp","lib/wide/gradient_2.bmp","lib/wide/gradient_3.bmp","lib/wide/gradient_4.bmp","lib/wide/gradient_5.bmp","lib/wide/gradient_6.bmp","lib/wide/gradient_7.bmp"]
files_h = ["lib/half/gradient_0.bmp","lib/half/gradient_1.bmp","lib/half/gradient_2.bmp","lib/half/gradient_3.bmp","lib/half/gradient_4.bmp","lib/half/gradient_5.bmp","lib/half/gradient_6.bmp","lib/half/gradient_7.bmp"]
#files = ["lib/all/1.bmp","lib/all/1.bmp","lib/all/1.bmp","lib/all/1.bmp","lib/all/5.bmp","lib/all/5.bmp","lib/all/5.bmp","lib/all/5.bmp"]
files = ["lib/paddedon.bmp"]*2 + ["lib/paddedoff.bmp"]*6
files1 = ["lib/all/1.bmp"]*3 + ["lib/all/5.bmp"]*5
files2 = ["lib/all/1.bmp"]*4 + ["lib/all/5.bmp"]*4
files3 = ["lib/all/5.bmp"]*8
d4 = D4100Client()

#loadImage(im, d4, 0)


'''
for i in range(100):
    print(i)
    im = np.zeros([1080,1920])
    im[i,480*0:480*1] = 255*np.ones([480])
    padded = 255*np.ones([1080,2048])
    padded[:,64:-64]=im
    #plt.imshow(padded)
    loadRaw(im,d4,0)
    #sleep(1)
'''
''
#imlist_n = initImageList(files_n)
imlist_w = initImageList(files_w)
imlist_h = initImageList(files_h)
imlist1 = initPadImageList(files)
imlist2 = initImageList(files1)
imlist3 = initImageList(files2)
imlist4 = initImageList(files3)
''
'''
#loadGray(imlist_w,d4,0)
#loadImage(files_h[-1],d4,0)
loadImage(im, d4, 0)
#loadImage("lib/test1.bmp",d4,0)
#loadImage(im2, d4, 0)

print('running loop')
for i in range(20):
    loadGray(imlist1,d4,0)
    print(i)
    loadGray(imlist4,d4,0)
    #loadGray(imlist2,d4,0)
    #loadGray(imlist3,d4,0)
'''