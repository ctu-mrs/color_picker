import numpy as np
import cv2 as cv
roi = cv.imread('/home/mrs/red_select.png')
hsv = cv.cvtColor(roi,cv.COLOR_BGR2LAB)
target = cv.imread('/home/mrs/red_box.png')
hsvt = cv.cvtColor(target,cv.COLOR_BGR2LAB)
# calculating object histogram
print(hsv.shape)
roihist = cv.calcHist([hsv],[0, 1,2], None, [225, 256, 223], [0, 225, 0, 256,0,223] )
print(roihist+roihist)
# normalize histogram and apply backprojection
cv.normalize(roihist,roihist,0,255,cv.NORM_MINMAX)
dst = cv.calcBackProject([hsvt],[0,1,2],roihist,[0,225,0,256,0,223],1)
# Now convolute with circular disc
disc = cv.getStructuringElement(cv.MORPH_ELLIPSE,(5,5))
cv.filter2D(dst,-1,disc,dst)
# threshold and binary AND
ret,thresh = cv.threshold(dst,50,255,0)
thresh = cv.merge((thresh,thresh,thresh))
res = cv.bitwise_and(target,thresh)
res = np.vstack((target,thresh,res))
cv.imwrite('/home/mrs/res.jpg',res)
