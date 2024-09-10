from src.D_Client import D4100Client
import matplotlib.pyplot as plt
import numpy as np
import pathlib
from time import sleep
from src.load_image import *


im = "lib/checker1.jpg"
im2 = "lib/line.bmp"
im3 = "lib/TI Logo.bmp"
files_w = ["lib/wide/gradient_0.bmp","lib/wide/gradient_1.bmp","lib/wide/gradient_2.bmp","lib/wide/gradient_3.bmp","lib/wide/gradient_4.bmp","lib/wide/gradient_5.bmp","lib/wide/gradient_6.bmp","lib/wide/gradient_7.bmp"]
files_h = ["lib/half/gradient_0.bmp","lib/half/gradient_1.bmp","lib/half/gradient_2.bmp","lib/half/gradient_3.bmp","lib/half/gradient_4.bmp","lib/half/gradient_5.bmp","lib/half/gradient_6.bmp","lib/half/gradient_7.bmp"]
snoopyfiles = ["lib/snoopy/0.bmp","lib/snoopy/1.bmp","lib/snoopy/2.bmp","lib/snoopy/3.bmp","lib/snoopy/4.bmp","lib/snoopy/5.bmp","lib/snoopy/6.bmp","lib/snoopy/7.bmp"]
d4 = D4100Client()

#loadImage(im, d4, 0)


imlist_w = initImageList(files_w, d4)
imlist_h = initImageList(files_h ,d4)
snoopy = initImageList(snoopyfiles ,d4)


#loadRaw(snoopy,d4, 0, 7)
#loadImage(im, d4, 0)
