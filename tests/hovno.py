import numpy as np
import cv2


samp = np.loadtxt('./sample.txt')
samp = np.reshape(samp, (180,255))
print("samp {}".format(samp.shape))

while(1):
    cv2.imshow('sample', samp)
    k = cv2.waitKey(33)
    if k==27:    # Esc key to stop
        break
    elif k==-1:  # normally -1 returned,so don't print it
        continue
    else:
        print k # else print its value
