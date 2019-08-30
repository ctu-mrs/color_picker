#!/usr/bin/env python
from __future__ import division
# #{ imports

from cv_bridge import CvBridge
from PIL import Image
import rospy
import copy
import time
import numpy as np
from math import hypot
import math
import cv2
import Tkinter
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import Image as RosImg
from math import hypot
from scipy.stats import norm
from scipy import ndimage
import pylab as plb
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy import asarray as ar,exp
from scipy.stats import circstd,circmean
from balloon_color_picker.srv import (
    Capture,
    CaptureResponse,
    ChangeSigma,
    ChangeSigmaResponse,
    ClearCount,
    ClearCountResponse,
    GetCount,
    GetCountResponse,
    ChangeSigmaLab,
    ChangeSigmaLabResponse,
    Save,
    SaveResponse,
    GetConfig,
    GetConfigResponse,
    Pic,
    PicResponse,
    Params,
    ParamsResponse
)
from balloon_color_picker.msg import (
    HistMsg
)
from std_msgs.msg import (
    Float64MultiArray,
    MultiArrayLayout
)

# #} end of imports

HSV = 0
LAB = 1

class ColorCapture():
    def __init__(self):
        rospy.init_node('color_picker_gui')

        self.circle_pub = rospy.Publisher('circle_topic', RosImg, queue_size=1)
        self.circle_hsv = rospy.Publisher('circle_hsv', RosImg, queue_size=1)
        self.circle_lab = rospy.Publisher('circle_lab', RosImg, queue_size=1)
        self.balloon_sub = rospy.Subscriber('image_topic', RosImg, self.balloon_call, queue_size = 1)
        self.balloon_filt  = rospy.Subscriber('image_topic', RosImg, self.balloon_filter, queue_size = 1)
        self.balloon_lab  = rospy.Subscriber('image_topic', RosImg, self.balloon_filter_lab, queue_size = 1)
        self.bridge = CvBridge()
        self.save_dir = rospy.get_param('~save_dir')
        self.config_path= rospy.get_param('~config_path')
        self.save_to_drone= rospy.get_param('~save_to_drone')


        self.cur_img = np.zeros([720,1280])

        self.services_ready = False
        self.sigma_multi_h = 3
        self.sigma_multi_s = 3
        self.sigma_multi_v = 3
        self.sigma_multi_l = 3
        self.sigma_multi_a = 3
        self.sigma_multi_b = 3
        self.img_count = 0

        ## hsv
        self.h_mean = 0
        self.h_sigma = 0
        self.s_mean = 0
        self.s_sigma = 0
        self.v_mean = 0
        self.v_sigma = 0

        self.h_arr = []
        self.s_arr = []
        self.v_arr = []
        self.hsv_roi = []
        self.hist_h = np.zeros([2,180])
        self.hist_h[1] = np.arange(180)
        self.hist_s = np.zeros([2,255])
        self.hist_s[1] = np.arange(255)
        self.hist_v = np.zeros([2,255])
        self.hist_v[1] = np.arange(255)

        ## lab
        self.l_mean = 0
        self.l_sigma = 0
        self.a_mean = 0
        self.a_sigma = 0
        self.b_mean = 0
        self.b_sigma = 0

        self.l_arr = []
        self.u_arr = []
        self.lv_arr = []
        self.lab_roi = []

        self.hist_l = np.zeros([2,255])
        self.hist_l[1] = np.arange(255)
        self.hist_a = np.zeros([2,256])
        self.hist_a[1] = np.arange(256)
        self.hist_b = np.zeros([2,223])
        self.hist_b[1] = np.arange(223)



    def balloon_call(self,data):

        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # img = img.copy()
        if self.services_ready == False:

            rospy.loginfo('first {}'.format(self.services_ready))

            self.capture_service = rospy.Service('capture', Capture, self.capture)
            self.sigma_service = rospy.Service('change_sigma', ChangeSigma,self.change_sigma)
            self.sigma_service = rospy.Service('change_sigma_lab', ChangeSigmaLab,self.change_sigma_lab)
            self.clear_service = rospy.Service('clear_count', ClearCount, self.clear_count)
            self.get_service  = rospy.Service('get_count', GetCount, self.get_count)
            self.config_service  = rospy.Service('get_config', GetConfig, self.get_config)
            self.pic_service  = rospy.Service('get_pic', Pic, self.save_pic)
            self.params_service  = rospy.Service('get_params', Params, self.get_params)
            self.prepare_mask(img)
            rospy.loginfo('Services started')
            self.services_ready = True
        self.cur_img = img.copy()
        img.setflags(write=1)
        x, y, c =img.shape
        cv  = (y//2, x//2)
        r = x//3
        cv2.circle(img, cv,r, 20 ,thickness=7, lineType=8, shift=0)
        # imgmsg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
        imgmsg = self.bridge.cv2_to_imgmsg(img)
        imgmsg.encoding = "bgr8"
        self.circle_pub.publish(imgmsg)


    def balloon_filter(self,data):

        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([self.h_mean-self.h_sigma*self.sigma_multi_h,self.s_mean-self.s_sigma*self.sigma_multi_s,self.v_mean-self.v_sigma*self.sigma_multi_v])
        upper = np.array([self.h_mean+self.h_sigma*self.sigma_multi_h,self.s_mean+self.s_sigma*self.sigma_multi_s,self.v_mean+self.v_sigma*self.sigma_multi_v])
        # Threshold the HSV image to get only blue colors
        if upper[0] > 180:
            dif = upper[0] - 180
            upper_1 = copy.copy(upper)
            upper_1[0] = 180
            upper_2 = copy.copy(upper)
            upper_2[0] = dif

            lower_2 = copy.copy(lower)
            lower_2[0] = 0
            mask_1 = cv2.inRange(hsv, lower, upper_1)
            mask_2 = cv2.inRange(hsv, lower_2, upper_2)

            # Bitwise-AND mask and original imagettttt
            mask = cv2.bitwise_or(mask_1,mask_2)

        elif lower[0] < 0:
            dif = 180 + lower[0]

            upper_1 = copy.copy(upper)
            lower_1 = copy.copy(lower)
            lower_1[0] = 0


            upper_2 = copy.copy(upper)
            lower_2 = copy.copy(lower)
            lower_2[0] = dif
            upper_2[0] = 180

            mask_1 = cv2.inRange(hsv, lower_1, upper_1)
            mask_2 = cv2.inRange(hsv, lower_2, upper_2)
            # Bitwise-AND mask and original imagettttt
            mask = cv2.bitwise_or(mask_1,mask_2)

        else:
            mask = cv2.inRange(hsv, lower, upper)

        self.lower_hsv = lower
        self.upper_hsv = upper
        # Bitwise-AND mask and original imagettttt
        res = cv2.bitwise_and(img,img, mask= mask)
        # msg = self.bridge.cv2_to_imgmsg(res,'rgb8')
        msg = self.bridge.cv2_to_imgmsg(res)
        msg.encoding = "bgr8"
        self.circle_hsv.publish(msg)

    def balloon_filter_lab(self,data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)

        lower = np.array([self.l_mean-self.l_sigma*self.sigma_multi_l,self.a_mean-self.a_sigma*self.sigma_multi_a,self.b_mean-self.b_sigma*self.sigma_multi_b])
        upper = np.array([self.l_mean+self.l_sigma*self.sigma_multi_l,self.a_mean+self.a_sigma*self.sigma_multi_a,self.b_mean+self.b_sigma*self.sigma_multi_b])
        # Threshold the HSV image to get only blue colors
        self.lower_lab = lower
        self.upper_lab = upper
        mask = cv2.inRange(hsv, lower, upper)

        # Bitwise-AND mask and original imagettttt
        res = cv2.bitwise_and(img,img, mask= mask)
        # msg = self.bridge.cv2_to_imgmsg(res,'rgb8')
        msg = self.bridge.cv2_to_imgmsg(res)
        msg.encoding = "bgr8"
        self.circle_lab.publish(msg)




    def capture(self,request):

        img = self.cur_img.copy()

        t = time.time()
        h,s,v = self.get_masked_colors(img, HSV)

        rospy.loginfo('time for hsv {}'.format(time.time() - t))
        t = time.time()

        l,a,b = self.get_masked_colors(img, LAB)

        rospy.loginfo('time for lab {}'.format(time.time() - t))

        t = time.time()
        res = self.np_hist(h,s,v, l, a, b)

        rospy.loginfo('time for np_hist {}'.format(time.time() - t))
        self.accumulate_hists(h,s,v,l,a,b)
        if self.img_count > 0:
            self.average(res)
        else:
            # self.hist_h[0],self.hist_h[1] = np.histogram(h, range=[0,180],bins=180)
            # self.hist_s[0],self.hist_s[1] = np.histogram(s, range=[0,255],bins=255)
            # self.hist_v[0],self.hist_v[1] = np.histogram(v, range=[0,255],bins=255)

            # self.hist_l[0],self.hist_l[1] = np.histogram(l, range=[0,255],bins=255)
            # self.hist_a[0],self.hist_a[1] = np.histogram(a, range=[0,255],bins=255)
            # self.hist_b[0],self.hist_b[1]= np.histogram(b, range=[0,255],bins=255)
            self.h_mean,self.h_sigma  = res[0]

            self.s_mean, self.s_sigma = res[1]
            self.v_mean, self.v_sigma = res[2]

            self.l_mean, self.l_sigma = res[3]
            self.a_mean, self.a_sigma = res[4]
            self.b_mean, self.b_sigma = res[5]


        self.img_count +=1
        rospy.loginfo('h {} s {} v {}'.format(self.h_mean, self.s_mean, self.v_mean))
        rospy.loginfo('l {} a {} b {}'.format(self.l_mean, self.a_mean, self.b_mean))
        return CaptureResponse(h,s,v,(self.h_mean, self.s_mean,self.v_mean, self.l_mean, self.a_mean, self.b_mean), (self.h_sigma,self.v_sigma,self.s_sigma, self.l_sigma, self.a_sigma,self.b_sigma), l, a, b, self.img_count)

    def change_sigma(self,request):
        self.sigma_multi_h = request.sigma_h
        self.sigma_multi_s = request.sigma_s
        self.sigma_multi_v = request.sigma_v
        rospy.loginfo('sigma hsv  {}'.format(request))
        return ChangeSigmaResponse(self.h_mean, self.s_mean, self.v_mean)

    def change_sigma_lab(self,request):
        self.sigma_multi_l = request.sigma_l
        self.sigma_multi_a= request.sigma_a
        self.sigma_multi_b = request.sigma_b
        rospy.loginfo('sigma lab {}'.format(request))
        return ChangeSigmaLabResponse(self.l_mean, self.a_mean, self.b_mean)

    def average(self,res):
        h_mean,h_sigma  = res[0]

        s_mean, s_sigma = res[1]
        v_mean, v_sigma = res[2]

        l_mean, l_sigma = res[3]
        u_mean, u_sigma = res[4]
        lv_mean, lv_sigma = res[5]

        self.h_mean,self.h_sigma  = self.av_std(self.h_mean, h_mean, self.h_sigma,h_sigma)

        self.s_mean, self.s_sigma = self.av_std(self.s_mean, s_mean, self.s_sigma,s_sigma)

        self.v_mean, self.v_sigma = self.av_std(self.v_mean, v_mean, self.v_sigma,v_sigma)
        self.l_mean, self.l_sigma = self.av_std(self.l_mean, l_mean, self.l_sigma,l_sigma)

        self.a_mean, self.a_sigma = self.av_std(self.a_mean, u_mean, self.a_sigma,u_sigma)
        self.b_mean, self.b_sigma = self.av_std(self.b_mean, lv_mean, self.b_sigma,lv_sigma)




    def av_std(self, mean_old, mean_new,std_old, std_new):

        res_m = (mean_old + mean_new)/2
        res_s = std_old**2 + std_new**2 + mean_old + mean_new - 2*res_m
        res_s = math.sqrt(res_s/2)
        return res_m, res_s

    def accumulate_hists(self, h,s,v,l,a,b):
        hist_h = np.histogram(h, range=[0,180],bins=180)
        hist_s = np.histogram(s, range=[0,255],bins=255)
        hist_v = np.histogram(v, range=[0,255],bins=255)

        hist_l = np.histogram(l, range=[0,255],bins=255)
        hist_a = np.histogram(a, range=[0,256],bins=256)
        hist_b = np.histogram(b, range=[0,223],bins=223)
        self.hist_h[0] += hist_h[0]
        self.hist_s[0] += hist_s[0]
        self.hist_v[0] += hist_v[0]

        self.hist_l[0] += hist_l[0]
        self.hist_a[0] += hist_a[0]
        self.hist_b[0] += hist_b[0]


    def clear_count(self,request):
        self.img_count = 0
        ## hsv
        self.h_mean = 0
        self.h_sigma = 0
        self.s_mean = 0
        self.s_sigma = 0
        self.v_mean = 0
        self.v_sigma = 0

        self.h_arr = []
        self.s_arr = []
        self.v_arr = []

        ## lab
        self.l_mean = 0
        self.l_sigma = 0
        self.a_mean = 0
        self.a_sigma = 0
        self.b_mean = 0
        self.b_sigma = 0

        self.l_arr = []
        self.u_arr = []
        self.lv_arr = []


        rospy.loginfo('cleared {}'.format(self.img_count))
        return ClearCountResponse()

    def get_count(self, request):
        return GetCountResponse(self.img_count)


    def np_hist(self,h,s,v,l,a,b):


        h_mean = circmean(h, high=180)
        h_sigma = circstd(h,high=180)

        s_mean, s_sigma = norm.fit(s)
        v_mean, v_sigma = norm.fit(v)

        l_mean, l_sigma = norm.fit(l)
        u_mean, u_sigma = norm.fit(a)
        lv_mean, lv_sigma = norm.fit(b)
        return  (h_mean, h_sigma), (s_mean,s_sigma), (v_mean,v_sigma), (l_mean, l_sigma), (u_mean, u_sigma), (lv_mean, lv_sigma)


    def get_masked_colors(self,img, color_space):

        if color_space == HSV:
            color = cv2.COLOR_BGR2HSV
        elif color_space == LAB:
            color = cv2.COLOR_BGR2Lab

        hsv = cv2.cvtColor(img, color)
        masked = hsv[self.mask==1, :]

        if color_space == HSV:
            
            if self.img_count > 0:
                self.hsv_roi +=cv2.calcHist([hsv],[0, 1,2], self.mask, [180, 256,255], [0, 180, 0, 256,0,255] )
            else:
                rospy.loginfo('shape {}'.format(masked.shape))
                self.hsv_roi = cv2.calcHist([hsv],[0, 1,2], self.mask, [180, 256,255], [0, 180, 0, 256,0,255] )
        elif color_space == LAB:
            if self.img_count > 0:
                self.lab_roi += cv2.calcHist([hsv],[0, 1,2], self.mask, [225, 256,223], [0, 225, 0, 256,0,223])
            else:
                rospy.loginfo('shape {}'.format(hsv.shape))
                self.lab_roi = cv2.calcHist([hsv],[0, 1,2], self.mask, [225, 256,223], [0, 225, 0, 256,0,223])
        a = masked[:, 0]
        b = masked[:, 1]
        c = masked[:, 2]
        return a,b,c

    def prepare_mask(self, img):
        x, y, c =img.shape
        cv  = (y//2, x//2)
        print(cv)
        r = x//3
        x,y = cv
        rows = img.shape[0]
        cols = img.shape[1]
        zero = np.zeros((img.shape[0], img.shape[1]),dtype="uint8")

        cv2.circle(zero, (x, y), r, 1, -1)
        self.mask = zero
    def get_config(self, req):

        hst_msg_h = HistMsg()
        hst_msg_h.height = self.hsv_roi.shape[0]
        hst_msg_h.width = self.hsv_roi.shape[1]
        rospy.loginfo('shape {}'.format(self.hsv_roi.shape))
        hst_msg_h.values = self.hsv_roi.flatten().tolist()
        
        np.savetxt('/home/mrs/git/testing_functions/balloon.txt', self.lab_roi.flatten())
        # np.savetxt('/home/mrs/git/testing_functions/balloon.txt', self.hsv_roi)
        
        return GetConfigResponse((self.h_mean, self.h_sigma*self.sigma_multi_h*2),
                                 (self.s_mean, self.s_sigma*self.sigma_multi_s*2),
                                 (self.v_mean, self.v_sigma*self.sigma_multi_v*2),
                                 (self.l_mean, self.l_sigma*self.sigma_multi_l*2),
                                 (self.a_mean, self.a_sigma*self.sigma_multi_a*2),
                                 (self.b_mean, self.b_sigma*self.sigma_multi_b*2),
                                 )

    def save_pic(self,req):
        rospy.loginfo('cur_img {}'.format(self.cur_img))
        rospy.loginfo('ing {}'.format(cv2.imwrite('/home/mrs/last_img.png',self.cur_img)))
        return PicResponse()

    def get_params(self, req):

        return ParamsResponse(self.config_path, self.save_dir, self.circle_pub.name, self.circle_hsv.name, self.circle_lab.name,self.save_to_drone)

if __name__ == '__main__':
    c = ColorCapture()
    rospy.spin()
    # menu()
