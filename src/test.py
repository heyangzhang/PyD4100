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

d4 = D4100Client()
im = "lib/checker1.jpg"
imarray = plt.imread(im)
imarray = imarray[:,:,0]
image = imarray.flatten().tolist()

d4.set_tpg_enable(0,0)
d4.clear_fifos(0)

blocks = 3
block_size = (1920*1080)//blocks

d4.set_row_mode(0,0b11)
d4.set_row_address(0,0)
d4.load3(0)

for i in range(0, blocks):
    d4.set_row_mode(0,0b01)
    d4.load3(0)
    data = image[i*block_size:(i+1)*block_size]
    if d4.load_data(0,data) == 0:
        raise Exception("didn't load")
    sleep(0.01)
d4.global_reset(0)