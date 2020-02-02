from __future__ import division
import numpy as np
# import matplotlib.pyplot as plt
import cv2
import matplotlib as mpl
from matplotlib import pyplot



def colors(img):

    # img = cv2.imread('../data/yellow_data.png')

    color = ('r','g','b')
    for i,col in enumerate(color):
        print(i)
        histr = cv2.calcHist([img],[i],None,[256],[0,256])
        # print(histr)
        plt.plot(histr,color = col)
        plt.xlim([0,256])
    plt.show()

def get_mask(img, x1,y1,x2,y2):

    print('x1 {} y1 {} x2 {} y2 {}'.format(x1,y1,x2,y2))
    mask = np.zeros([img.shape[0], img.shape[1]])
    if x1>x2:
        if y1>y2:
            mask[x2:x1,y2:y1] = 1
        elif y1<y2:
            mask[x2:x1,y1:y2] = 1
    elif x1<x2:
        if y1>y2:
            mask[x1:x2,y2:y1] = 1
        elif y1<y2:
            mask[x1:x2,y1:y2] = 1

    return mask






im = cv2.imread("./sample.txt")
mask = get_mask(im,658, 633,726,733)

# x1 658 y1 633 x2 726 y2 733 
# colors(im)
hsv = np.zeros(im.shape)
print('im {}'.format(im.shape))
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
hbin = 180
sbin = 255 
histSize = [hbin, sbin]
hrange = [0,180]
srange = [0,256]

ranges = [hrange, srange]

channels = [0,1]

hist = cv2.calcHist(
                    images=[hsv],
                    channels=channels,
                    mask = mask.astype('uint8'),
                    histSize = histSize,
                    ranges = [0,180,0,256]
                    )

print("hist shape {}".format(hist.flatten().aslist()))


minVal, maxVal, l, m = cv2.minMaxLoc(hist)
print("min {} max {}".format(minVal, maxVal))
hist = (hist-minVal)/(maxVal-minVal)*255.0

# #{ 

# scale = 20
# hstImg = np.zeros([sbin*scale, hbin*scale, 3])



# print(maxVal)
# for h in range(hbin):
#     for s in range(sbin):
#         binVal = hist[h,s]
#         intensity = int(binVal*255/maxVal)
#         # print(intensity)
#         start = (h*scale, s*scale)
#         end = ((h+1)*scale -1 , (s+1)*scale-1)
#         # print("start {} end {} intensity {}".format(start,end, intensity))
#         color = (intensity,intensity,intensity)
#         thickness = 1
#
          # cv2.rectangle(hstImg, start,end,color, cv2.FILLED)
# #} end of

# make values from -5 to 5, for this example

fig = pyplot.figure(1)

# make a color map of fixed colors

cmap2 = mpl.colors.LinearSegmentedColormap.from_list('my_colormap',
                                                     ['blue','black','red'],
                                                     256)
bounds=range(20)

# tell imshow about color map so that only set colors are used
img = pyplot.imshow(hist,interpolation='nearest',
                    cmap = cmap2,origin='lower')


pyplot.colorbar(img,cmap=cmap2)

pyplot.show()
# while(1):
#     cv2.imshow('img',hist)
#     k = cv2.waitKey(33)
#     if k==27:    # Esc key to stop
#         break
#     elif k==-1:  # normally -1 returned,so don't print it
#         continue
#     else:
#         print k # else print its value
