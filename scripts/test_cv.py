from __future__ import division
from math import hypot
from scipy.stats import norm
import numpy as np
from scipy import ndimage
import cv2
import pylab as plb
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy import asarray as ar,exp
from scipy.stats import circstd, circmean






def main():
    img = cv2.imread('../data/cricle2.png')
    img = cv2.flip(img, 0)
    print(img)
    x, y, c =img.shape
    print(img.shape)
    cv  = (y//2, x//2)
    print(cv)
    center = (756,359)

    mask = np.zeros(img.shape)
    cv2.circle(mask, cv,x//3, 20 ,thickness=7, lineType=8, shift=0)
    p = np.where(mask ==20)
    print(p)
    r = x//3
    x,y = cv
    rows = img.shape[0]
    cols = img.shape[1]

    for i in range(cols):
        for j in range(rows):
            if hypot(i-x, j-y) > r:
                img[j,i] = 0

    cv2.imwrite("../data/circle_2.png",img)
    cv2.imshow('image',img)
    # print(mask)
    while(1):
        k = cv2.waitKey(33)
        if k == ord('a'):
           print("a")
           
           cv2.destroyAllWindows()
           break
      
def colors():

    img = cv2.imread('../data/yellow_data.png')

    color = ('r','g','b')
    for i,col in enumerate(color):
        print(i)
        histr = cv2.calcHist([img],[i],None,[256],[0,256])
        print(histr)
        plt.plot(histr,color = col)
        plt.xlim([0,256])
    plt.show()

def get_color():

    img = cv2.imread('../data/green_data.png')

    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
    # numpy_vertical = np.hstack((img, hsv))
    # cv2.imshow('img', numpy_vertical)
    # print(hsv)
    # while(1):
    #     k = cv2.waitKey(33)
    #     if k == ord('a'):
    #        print("a")
           
    #        cv2.destroyAllWindows()
    #        break
    x, y, c =img.shape
    print(img.shape)
    cv  = (y//2, x//2)
    print(cv)
    r = x//3
    x,y = cv
    rows = img.shape[0]
    cols = img.shape[1]
    count = 0
    min_h = float('inf') 
    min_s = float('inf')
    min_v = float('inf')
    max_h = 0
    max_s = 0
    max_v = 0
    h_v = 0
    s_v = 0
    v_v = 0
    h_arr = []
    s_arr = []
    v_arr = []

    for i in range(cols):
        for j in range(rows):
            if hypot(i-x, j-y) < r-6.5:

                h,s,v = hsv[j,i]
                if h > max_h:
                    max_h = h
                elif h < min_h:
                    min_h = h
                if s > max_s:
                    max_s = s
                elif s < min_s:
                    min_s = s
                if v > max_v:
                    max_v = v
                elif v < min_v:
                    min_v = v


                h_v += h
                s_v += s
                v_v += v
                h_arr.append(h)                                                          
                s_arr.append(s)                                                          
                v_arr.append(v)                                                         
                count +=1
    print('h {}, s {}, v {}'.format(h_v/count, s_v/count, v_v/count))
    print('h max {}, h min {}'.format(max_h, min_h))
    print('s max {}, s min {}'.format(max_s, min_s))
    print('v max {}, v min {}'.format(max_v, min_v))
    print(count)
    # np.savetxt('../data/h.txt', h_arr)
    # np.savetxt('../data/s.txt', s_arr)
    # np.savetxt('../data/v.txt', v_arr)
    plt.figure(1)
    plt.subplot(221)
    plt.hist(h_arr, facecolor='r')
    # plt.xlim([0, 180])
    # plt.subplot(222)
    # plt.hist(s_arr, facecolor='g')
    # plt.xlim([0, 255])
    # plt.subplot(223)
    # plt.hist(v_arr, facecolor='b')
    # plt.xlim([0, 255])

    

    plt.show()

    # filter_colors(np.array([min_h,min_s,min_v]), np.array([max_h,max_s,max_v]), '../data/green_data.png')


def gaus(x,a,x0,sigma):
    return a*exp(-(x-x0)**2/(2*sigma**2))


def fit_gaus():
    data = np.loadtxt('../data/h.txt')
    plt.figure(1)
    plt.xlim([0, 180])
    d = plt.hist(data)
    plt.show()
    print(d[0].shape)
    print(d[1].shape)
    x = d[0]
    y = d[1]
    n = len(x)
    mean = sum(x*y)/n
    sigma = sum(y*(x-mean)**2)/n

    def gaus(x,a,x0,sigma):
        return a*exp(-(x-x0)**2/(2*sigma**2))

    popt,pcov = curve_fit(gaus,x,y,p0=[1,mean,sigma])

    plt.plot(x,y,'b+:',label='data')
    plt.plot(x,gaus(x,*popt),'ro:',label='fit')
    plt.show()
    # print("x shape {}, y shape {}".format(x.shape,y.shape))
    # x = ar(range(10))
    # y = ar([0,1,2,3,4,5,4,3,2,1])




def filter_colors(lower=None, upper=None, img_str=None):
    if img_str == None:
        img_str = '../data/circle.png'
    img = cv2.imread(img_str)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    
    if lower is None and upper is None:
        lower= np.array([98,97,20])
        upper= np.array([104,255,215])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)

    # Bitwise-AND mask and original imagettttt
    res = cv2.bitwise_and(img,img, mask= mask)

    # cv2.imshow('frame',res)
    numpy_horizontal = np.hstack((hsv, res))
    cv2.imshow('res',numpy_horizontal)
    k = cv2.waitKey(5) & 0xFF
    while(1):
        k = cv2.waitKey(33)
        if k == ord('a'):
           print("a")
           
           cv2.destroyAllWindows()
           break
 

def np_hist():
    h = np.loadtxt('../data/h.txt')
    s = np.loadtxt('../data/s.txt')
    v = np.loadtxt('../data/v.txt')
    print("loaded")

    # mean_h, std_h = norm.fit(h)
    mean_s, std_s = norm.fit(s)
    mean_v, std_v = norm.fit(v)
    plt.figure(1)
    plt.subplot(221)
    plt.hist(h, normed=True)
    xmin, xmax = (0,180)

    mean_h = 174
    std_h = 2.6
    x = np.linspace(xmin, xmax, 180)
    y = norm.pdf(x,mean_h,std_h)
    plt.plot(x, y)
    plt.subplot(222)
    plt.hist(s, normed=True)
    xmin, xmax = (0,256)
    x = np.linspace(xmin, xmax, 256)
    y = norm.pdf(x,mean_s,std_s)
    plt.plot(x, y)
    plt.subplot(223)
    plt.hist(v, normed=True)
    xmin, xmax = (0,223)
    x = np.linspace(xmin, xmax, 223)
    y = norm.pdf(x,mean_v,std_v)
    plt.plot(x, y)

    plt.show()

    sigma_multi = 15

    print('mean ha {} sigma {}'.format(mean_h, std_h))
    upper = np.array([mean_h+sigma_multi*std_h, mean_s+sigma_multi*std_s, mean_v+sigma_multi*std_v])
    lower = np.array([mean_h-sigma_multi*std_h, mean_s-sigma_multi*std_s, mean_v-sigma_multi*std_v])
    print('upper {}, lower {}'.format(upper,lower))
    filter_colors(lower=lower, upper=upper, img_str='../data/violet.png')

def get_indexes():
    im1 = cv2.imread('../data/green_data.png')
    im1 = cv2.cvtColor(im1, cv2.COLOR_BGR2HSV)
    # im1 = cv2.imread('../data/yellow2.png')
    x,y,c = im1.shape
    print(im1.shape)
    zero = np.zeros((im1.shape[0], im1.shape[1]))
    r = x//3
    x,y = y//2,x//2
    # rows = zero.shape[0]
    # cols = zero.shape[1]
    # indexes = []
    # for i in range(cols):
    #     for j in range(rows):
    #         if hypot(i-x, j-y) < r-6.5:
    #             indexes.append((j,i))
    #             if j == 720:
    #                 print('yes')

    #             zero[j,i] = 1
    #             f = im1[j,i][0]
    cv2.circle(zero, (x, y), r, 1, -1)

    cv2.imshow('image',zero)
    cv2.waitKey(0)
    mask = zero == 1
    z = im1[mask, :]
    hues = z[:, 0]
    sats = z[:, 1]
    vals = z[:, 2]
    # d = cv2.bitwise_and(im1,im1, mask=zero.astype(np.int))
    print('mean {} max {} min {}'.format(np.mean(hues), max(hues), min(hues)))
    print('mean {} max {} min {}'.format(np.mean(sats), max(sats), min(sats)))
    print('mean {} max {} min {}'.format(np.mean(vals), max(vals), min(vals)))
    
    plt.hist(hues)
    plt.hist(sats)
    plt.hist(vals)
    plt.show()
    print(z)
    # im1 = cv2.imread('../data/green_data.png')
    # print(len(indexes))
    # h = im1[indexes[:160108]]
    # print(indexes)

    # print(f[0])
    # print(mask)

    while(1):
        k = cv2.waitKey(33)
        if k == ord('a'):
           print("a")
           
           cv2.destroyAllWindows()
           break

def test_mean():
    im1 = cv2.imread('../data/green_data.png')

    im1 = cv2.cvtColor(im1, cv2.COLOR_BGR2Lab)


    # im1 = cv2.imread('../data/yellow2.png')
    x,y,c = im1.shape
    zero = np.ones(im1.shape)
    r = x//3
    x,y = y//2,x//2
    rows = im1.shape[0]
    cols = im1.shape[1]

    for i in range(cols):
        for j in range(rows):
            if hypot(i-x, j-y) > r:
                zero[j,i] = 0

    c = im1.copy()
    c[zero==0] = 0
     
    # print(coord[0].shape)
    return
    # print(coord[1].shape)
    h = c[coord[0], coord[1]]
    print(h.shape)
    l_m = np.max(h[0])
    a_m = np.max(h[1])
    b_m = np.max(h[2])
    print(l_m)
    print(a_m)
    print(b_m)
    # print(ind)
    # f = im1[ind]
    cv2.imshow('image',c)
    # print(mask)

    while(1):
        k = cv2.waitKey(33)
        if k == ord('a'):
           print("a")
           
           cv2.destroyAllWindows()
           break
      
   
def tst_index():
    im1 = cv2.imread('../data/green_data.png')
    ind = np.loadtxt('../data/ind.txt')

    h = im1[ind][0]
    print(h)

def test_data():

    h = np.loadtxt('../data/h.txt')

    for i in range(len(h)):
        if h[i] < 90:
            h[i] +=90
        else:
            h[i] -=90
        
    mean = np.mean(h)
    
    
    mean_h, std_h = norm.fit(h)
    if mean_h > 90:
        mean = mean - 90
    else :
        mean +=90


    print("min {} max {} mean {} std {}".format(min(h), max(h), mean, std_h))
    plt.hist(h)
    plt.xlim([0, 180])
    plt.show()
    # np.savetxt('../data/h_new.txt', h)



def double_range():

    upper = [201,154.33688367,189.60092747]
    lower = [131,5.72783392, 48.33302417]

    im = cv2.imread('../data/violet.png')

    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    if upper[0] > 180:
        upper_h = 180
        upper_h2 = upper[0] - 180

    rows = im.shape[0]
    cols = im.shape[1]

    for i in range(cols):
        for j in range(rows):
            h,s,v = hsv[j,i]
            
            h_true = False
            s_true = False
            v_true = False
            if h > lower[0] or  h < upper_h2:
                h_true = True
            if s > lower[1] and s < upper[1]:
                s_true = True
            if v > lower[2] and v < upper[2]:
                v_true = True

            if (h_true is False) or (s_true is False) or  (v_true is False):
                im[j,i] = [0,0,0]
    cv2.imshow('image',im)
    # print(mask)

    while(1):
        k = cv2.waitKey(33)
        if k == ord('a'):
           print("a")
           
           cv2.destroyAllWindows()
           break
                
    return 


def test_circ():

    h = np.loadtxt('../data/h.txt')
    # h = h*2
    m,s = norm.fit(h)
    print('default mean {} std {}'.format(m,s))
    c_m = circmean(h)
    c_s = circstd(h)
    c_mh = circmean(h, high=180)
    c_sh = circstd(h, high=180)
    print('default circ mean {} std {}'.format(circmean(h), circstd(h)))
    print('mean {} std {}'.format(circmean(h, high=180), circstd(h, high=180)))
    xmin, xmax = (0,180)
    x = np.linspace(xmin, xmax, 180)
    y = norm.pdf(x,m,s)
    y_circ = norm.pdf(x,c_m,c_s)
    y_circh = norm.pdf(x,c_mh,c_sh)
    y_flip = norm.pdf(x,174.23,2.64)
    fl = plt.plot(x, y_flip, label='With flip ')
    l = plt.plot(x, y, label="Default mean")
    c = plt.plot(x, y_circ, label='Circular mean')
    ch = plt.plot(x, y_circh, label='Circular with limit')
    v = plt.hist(h, normed=True, label='Hue')
    plt.legend(loc='upper right')
    # plt.legend((l), ('mean'))
    # plt.legend((l,c,ch), ('Default mean', 'Circuled', 'Circlued with high 180'))
    plt.show()


def test_high():
    upper = np.array([8,154.33688367,189.60092747])
    lower = np.array([3,5.72783392, 48.33302417])

    im = cv2.imread('../data/violet.png')

    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    # if upper[0] > 180:
    #     upper_h = 180
    #     upper[0] = 180 - upper[0]

    rows = im.shape[0]
    cols = im.shape[1]

    print('u {}'.format(upper))
    mask = cv2.inRange(hsv, lower, upper)

    # Bitwise-AND mask and original imagettttt
    res = cv2.bitwise_and(im,im, mask= mask)

    # cv2.imshow('frame',res)


    cv2.imshow('image',res)
    # print(mask)

    while(1):
        k = cv2.waitKey(33)
        if k == ord('a'):
           print("a")
           
           cv2.destroyAllWindows()
           break
                
    return 



def test_mask():
    upper = np.array([201,154.33688367,189.60092747])
    lower = np.array([131,5.72783392, 48.33302417])


    upper_1 = np.array([180,154.33688367,189.60092747])
    lower_1 = np.array([131,5.72783392, 48.33302417])

    upper_2 = np.array([21,154.33688367,189.60092747])
    lower_2 = np.array([0,5.72783392, 48.33302417])
    im = cv2.imread('../data/violet.png')

    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)



    mask_1 = cv2.inRange(hsv, lower_1, upper_1)
    mask_2 = cv2.inRange(hsv, lower_2, upper_2)

    
    mask_res = cv2.bitwise_or(mask_1,mask_2)

    res = cv2.bitwise_and(im,im, mask= mask_res)
 
    masks = np.hstack((mask_2, res))
    cv2.imshow('image',masks)
    # print(mask)

    while(1):
        k = cv2.waitKey(33)
        if k == ord('a'):
           print("a")
           
           cv2.destroyAllWindows()
           break
                
    return 



# main()
# fit_gaus()
# get_color()
# filter_colors()
# np_hist()
# test_mean()
get_indexes()
# tst_index()
# test_data()
# double_range()
# test_mask()
# test_circ()
# test_high()
