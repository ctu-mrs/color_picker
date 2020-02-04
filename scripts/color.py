#!/usr/bin/env python
from __future__ import division
# #{ imports

from cv_bridge import CvBridge
from PIL import Image
import rospy
import copy
import os
import time
import binascii
import yaml
from io import BytesIO
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
    CaptureCropped,
    CaptureCroppedResponse,
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
    ParamsResponse,
    Freeze,
    FreezeResponse,
    UpdateObd,
    UpdateObdResponse,
    ChangeCallback,
    ChangeCallbackResponse,
    CaptureHist,
    CaptureHistResponse,
)
from std_srvs.srv import (
    Trigger,
    TriggerResponse
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
RAW = 2
BOTH = 3

class ColorCapture():

# #{ __init__

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
        self.circled_name = rospy.get_param('~circled')
        self.circled_hsv_name = rospy.get_param('~circled_hsv')
        self.circled_lab_name = rospy.get_param('~circled_lab')
        self.object_detect_name = rospy.get_param('~object_detect')

        ## | --------------------- set HSV params --------------------- |

        self.obd_h_c = rospy.get_param("~hue_center")
        self.obd_h_r = rospy.get_param("~hue_range")
        self.obd_s_c = rospy.get_param("~sat_center")
        self.obd_s_r = rospy.get_param("~sat_range")
        self.obd_v_c = rospy.get_param("~val_center")
        self.obd_v_s = rospy.get_param("~val_range")

        ## | --------------------- get HSV params --------------------- |

        self.obd_l_c = rospy.get_param("~l_center")
        self.obd_l_r = rospy.get_param("~l_range")
        self.obd_a_c = rospy.get_param("~a_center")
        self.obd_a_r = rospy.get_param("~a_range")
        self.obd_b_c = rospy.get_param("~b_center")
        self.obd_b_r = rospy.get_param("~b_range")
        self.obd_segment = rospy.get_param("~segment_name")
        self.ball_size = rospy.get_param("~ball_size")




        self.cur_img = None
        self.circle_img = None
        # self.circle_img = np.zeros([720,1280,3])

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
        self.hist_hs = None

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
        self.hist_a = np.zeros([2,255])
        self.hist_a[1] = np.arange(255)
        self.hist_b = np.zeros([2,255])
        self.hist_b[1] = np.arange(255)
        self.freeze = False
        self.color_space = 'HSV'
        self.sub = RAW




# #} end of __init__

# #{ balloon_img_callback

    def balloon_call(self,data):

        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # img = img.copy()
        if self.services_ready == False:

            rospy.loginfo('First message in, services status: {}'.format(self.services_ready))

            self.capture_service = rospy.Service('capture', Capture, self.capture)
            self.capture_cropped_service = rospy.Service('capture_cropped', CaptureCropped, self.capture_cropped)
            self.sigma_service = rospy.Service('change_sigma', ChangeSigma,self.change_sigma)
            self.sigma_service = rospy.Service('change_sigma_lab', ChangeSigmaLab,self.change_sigma_lab)
            self.clear_service = rospy.Service('clear_count', ClearCount, self.clear_count)
            self.get_service  = rospy.Service('get_count', GetCount, self.get_count)
            self.config_service  = rospy.Service('get_config', GetConfig, self.get_config)
            self.object_service  = rospy.Service('change_obd', UpdateObd, self.set_object_detect)
            self.object_update  = rospy.ServiceProxy('update_obd', Trigger)
            self.pic_service  = rospy.Service('get_pic', Pic, self.save_pic)
            self.freeze_service  = rospy.Service('freeze', Freeze, self.freeze_callback)
            self.params_service  = rospy.Service('get_params', Params, self.get_params)
            self.callback_service = rospy.Service('change_callback', ChangeCallback, self.change_callback)
            self.capture_hist = rospy.Service('capture_hist', CaptureHist, self.create_hist)
            self.prepare_mask(img)
            rospy.loginfo('Services started')
            self.services_ready = True
        x, y, c = img.shape
        cv  = (y//2, x//2)
        r = x//3
        if not self.freeze:
            self.cur_img = img.copy()
            self.circle_img = img.copy()
            self.pub_img = img.copy
        # cv2.circle(self.circle_img, cv,r, 20 ,thickness=7, lineType=8, shift=0)
        # imgmsg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
        if self.sub != RAW:
            return
        imgmsg = self.bridge.cv2_to_imgmsg(self.circle_img)
        imgmsg.encoding = "bgr8"


        self.circle_pub.publish(imgmsg)

# #} end of balloon_img_callback

# #{ balloon_filtered_hsv_callback

    def balloon_filter(self,data):
        if self.sub != HSV:
            return

        img = np.array(self.cur_img.copy(), dtype=np.uint8)
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
        white = 255*np.ones_like(img)
        # Bitwise-AND mask and original imagettttt
        res = cv2.bitwise_and(white,white, mask=mask)
        # rospy.loginfo(' res {}'.format(res[res==0]))
        
        # msg = self.bridge.cv2_to_imgmsg(res,'rgb8')
        msg = self.bridge.cv2_to_imgmsg(res)
        msg.encoding = "bgr8"
        self.circle_hsv.publish(msg)


# #} end of balloon_filtered_hsv_callback

# #{ balloon_filter_lab_callback 

    def balloon_filter_lab(self,data):
        if self.sub != LAB:
            return


        # img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img = np.array(self.cur_img.copy(), dtype=np.uint8)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)

        lower = np.array([self.l_mean-self.l_sigma*self.sigma_multi_l,self.a_mean-self.a_sigma*self.sigma_multi_a,self.b_mean-self.b_sigma*self.sigma_multi_b])
        upper = np.array([self.l_mean+self.l_sigma*self.sigma_multi_l,self.a_mean+self.a_sigma*self.sigma_multi_a,self.b_mean+self.b_sigma*self.sigma_multi_b])
        # Threshold the HSV image to get only blue colors
        self.lower_lab = lower
        self.upper_lab = upper
        mask = cv2.inRange(hsv, lower, upper)

        # Bitwise-AND mask and original imagettttt
        # res = cv2.bitwise_and(img,img, mask= mask)
        white = 255*np.ones_like(img)
        # Bitwise-AND mask and original imagettttt
        res = cv2.bitwise_and(white,white, mask=mask)
        # msg = self.bridge.cv2_to_imgmsg(res,'rgb8')
        msg = self.bridge.cv2_to_imgmsg(res)
        msg.encoding = "bgr8"
        self.circle_lab.publish(msg)



# #} end of balloon_filter_lab_callback 

# #{ capture_service_callback


    def capture(self,request):

        img = self.cur_img.copy()


        rospy.loginfo('Capture circle service called')
        t = time.time()
        h,s,v = self.get_masked_colors(img, HSV,self.mask)

        rospy.loginfo('time for hsv {}'.format(time.time() - t))
        t = time.time()

        l,a,b = self.get_masked_colors(img, LAB,self.mask)

        rospy.loginfo('time for lab {}'.format(time.time() - t))

        t = time.time()
        res = self.np_hist(h,s,v, l, a, b)


        rospy.loginfo('time for np_hist {}'.format(time.time() - t))
        self.accumulate_hists(h,s,v,l,a,b)
        if self.img_count > 0:
            self.average(res)
        else:
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


# #} end of capture_service_callback

# #{ capture_cropped_service_callback

    def capture_cropped(self,request):
        x1 = request.x1
        y1 = request.y1
        x2 = request.x2
        y2 = request.y2

        rospy.loginfo('Cropped service called')
        mask = self.get_mask(x1,y1,x2,y2)

        img = self.cur_img.copy()

        t = time.time()
        h,s,v = self.get_masked_colors(img, HSV, mask)

        rospy.loginfo('time for hsv {}'.format(time.time() - t))
        t = time.time()

        l,a,b = self.get_masked_colors(img, LAB,mask)

        rospy.loginfo('time for lab {}'.format(time.time() - t))

        t = time.time()
        res = self.np_hist(h,s,v, l, a, b)

        rospy.loginfo('time for np_hist {}'.format(time.time() - t))
        self.accumulate_hists(h,s,v,l,a,b)
        if math.isnan(res[0][0]) or math.isnan(res[1][0]) or math.isnan(res[2][0]) or math.isnan(res[3][0]) or math.isnan(res[4][0]) or math.isnan(res[5][0]):
            rospy.loginfo('detected NaNs in res, abort')
            res = CaptureCroppedResponse()
            res.success = False
            return res

        if self.img_count > 0:
            self.average(res)
        else:
            self.h_mean,self.h_sigma  = res[0]
            self.s_mean, self.s_sigma = res[1]
            self.v_mean, self.v_sigma = res[2]

            self.l_mean, self.l_sigma = res[3]
            self.a_mean, self.a_sigma = res[4]
            self.b_mean, self.b_sigma = res[5]


        self.img_count +=1
        rospy.loginfo('h {} s {} v {}'.format(self.h_mean, self.s_mean, self.v_mean))
        rospy.loginfo('l {} a {} b {}'.format(self.l_mean, self.a_mean, self.b_mean))
        return CaptureCroppedResponse(True,h,s,v,(self.h_mean, self.s_mean,self.v_mean, self.l_mean, self.a_mean, self.b_mean), (self.h_sigma,self.v_sigma,self.s_sigma, self.l_sigma, self.a_sigma,self.b_sigma), l, a, b, self.img_count)


# #} end of capture_cropped_service_callback

# #{ change_sigma_callback

    def change_sigma(self,request):
        self.sigma_multi_h = request.sigma_h
        self.sigma_multi_s = request.sigma_s
        self.sigma_multi_v = request.sigma_v
        rospy.loginfo('sigma hsv  {}'.format(request))
        return ChangeSigmaResponse(self.h_mean, self.s_mean, self.v_mean)


# #} end of change_sigma

# #{ change_sigma_lab_callback

    def change_sigma_lab(self,request):
        self.sigma_multi_l = request.sigma_l
        self.sigma_multi_a= request.sigma_a
        self.sigma_multi_b = request.sigma_b
        rospy.loginfo('sigma lab {}'.format(request))
        return ChangeSigmaLabResponse(self.l_mean, self.a_mean, self.b_mean)


# #} end of change_sigma_lab_callback

# #{ average_values

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



# #} end of average_values

# #{ average_std


    def av_std(self, mean_old, mean_new,std_old, std_new):

        res_m = (mean_old + mean_new)/2
        res_s = std_old**2 + std_new**2 + mean_old + mean_new - 2*res_m
        res_s = math.sqrt(res_s/2)
        return res_m, res_s


# #} end of average_std

# #{ accumulate_hists

    def accumulate_hists(self, h,s,v,l,a,b):
        hist_h = np.histogram(h, range=[0,180],bins=180)
        hist_s = np.histogram(s, range=[0,255],bins=255)
        hist_v = np.histogram(v, range=[0,255],bins=255)

        hist_l = np.histogram(l, range=[0,255],bins=255)
        hist_a = np.histogram(a, range=[0,255],bins=255)
        hist_b = np.histogram(b, range=[0,255],bins=255)
        self.hist_h[0] += hist_h[0]
        self.hist_s[0] += hist_s[0]
        self.hist_v[0] += hist_v[0]

        self.hist_l[0] += hist_l[0]
        self.hist_a[0] += hist_a[0]
        self.hist_b[0] += hist_b[0]


# #} end of accumulate_hists

# #{ clear_count_callback

    def clear_count(self,request):
        self.img_count = 0
        ## hsv
        self.h_mean = 0
        self.h_sigma = 0
        self.s_mean = 0
        self.s_sigma = 0
        self.v_mean = 0
        self.v_sigma = 0

        self.hist_hs = None
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


# #} end of clear_count_callback

# #{ get_count

    def get_count(self, request):
        return GetCountResponse(self.img_count)


# #} end of get_count

# #{ np_histogram generator

    def np_hist(self,h,s,v,l,a,b):


        h_mean = circmean(h, high=180)
        h_sigma = circstd(h,high=180)

        s_mean, s_sigma = norm.fit(s)
        v_mean, v_sigma = norm.fit(v)

        l_mean, l_sigma = norm.fit(l)
        u_mean, u_sigma = norm.fit(a)
        lv_mean, lv_sigma = norm.fit(b)
        return  (h_mean, h_sigma), (s_mean,s_sigma), (v_mean,v_sigma), (l_mean, l_sigma), (u_mean, u_sigma), (lv_mean, lv_sigma)


# #} end of np_histogram generator

# #{ get_masked_colors

    def get_masked_colors(self,img, color_space, mask=None):

        if color_space == HSV:
            color = cv2.COLOR_BGR2HSV
        elif color_space == LAB:
            color = cv2.COLOR_BGR2Lab

        hsv = cv2.cvtColor(img, color)


        masked = hsv[mask==1, :]

        if color_space == HSV:
            
            if self.img_count > 0:
                self.hsv_roi +=cv2.calcHist(images=[hsv],channels=[0, 1,2], mask=self.mask, histSize=[180, 255,255], ranges=[0, 180, 0, 255,0,255] )
            else:
                rospy.loginfo('shape {}'.format(masked.shape))
                self.hsv_roi = cv2.calcHist(images=[hsv],channels=[0, 1,2], mask=self.mask, histSize=[180, 255,255], ranges=[0, 180, 0, 255,0,255] )
        elif color_space == LAB:
            if self.img_count > 0:
                self.lab_roi += cv2.calcHist(images=[hsv],channels=[0, 1,2], mask=self.mask, histSize=[255, 255,255], ranges=[0, 255, 0, 255,0,255])
            else:
                rospy.loginfo('shape {}'.format(hsv.shape))
                self.lab_roi = cv2.calcHist(images=[hsv],channels=[0, 1,2], mask=self.mask, histSize=[255, 255,255], ranges=[0, 255, 0, 255,0,255])
        a = masked[:, 0]
        b = masked[:, 1]
        c = masked[:, 2]
        return a,b,c


# #} end of get_masked_colors

# #{ prepare_mask

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


# #} end of prepare_mask

# #{ get_mask

    def get_mask(self,x1,y1,x2,y2):

        rospy.loginfo('x1 {} y1 {} x2 {} y2 {}'.format(x1,y1,x2,y2))
        mask = np.zeros([self.cur_img.shape[0], self.cur_img.shape[1]])
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
 

# #} end of get_mask       

# #{ get_config

    def get_config(self, req):

        color_obj = {}
        conf_obj = {}

        #HSV
        hsv = {}
        hsv['hue_center'] = float(self.h_mean)
        hsv['hue_range'] = float(self.h_sigma*self.sigma_multi_h*2)
        hsv['sat_center'] = float(self.s_mean)
        hsv['sat_range'] = float(self.s_sigma*self.sigma_multi_s*2)
        hsv['val_center'] = float(self.v_mean)
        hsv['val_range'] = float(self.v_sigma*self.sigma_multi_v*2)
        conf_obj['hsv'] = hsv
        #LAB
        lab = {}
        lab['l_center'] = float(self.l_mean)
        lab['l_range'] = float(self.l_sigma*self.sigma_multi_l*2)
        lab['a_center'] = float(self.a_mean )
        lab['a_range'] = float(self.a_sigma*self.sigma_multi_a*2)
        lab['b_center'] = float(self.b_mean)
        lab['b_range'] = float(self.b_sigma*self.sigma_multi_b*2)
        conf_obj['lab'] = lab

        conf_obj['binarization_method_name'] = req.color_space.data

        conf_obj['physical_diameter'] = float(req.rad.data)
        color_name = os.path.basename(req.name.data).lower().split('.')[0]
        conf_obj['segment_color_name'] = color_name
        color_obj['ball'] = conf_obj
        if  os.path.isdir(req.name.data) is not True:
            f = file(req.name.data,'w')
            yaml.safe_dump(color_obj,f)
            
            rospy.loginfo('saved to dir {}'.format(req.name.data))
        
        return GetConfigResponse((self.h_mean, self.h_sigma*self.sigma_multi_h*2),
                                 (self.s_mean, self.s_sigma*self.sigma_multi_s*2),
                                 (self.v_mean, self.v_sigma*self.sigma_multi_v*2),
                                 (self.l_mean, self.l_sigma*self.sigma_multi_l*2),
                                 (self.a_mean, self.a_sigma*self.sigma_multi_a*2),
                                 (self.b_mean, self.b_sigma*self.sigma_multi_b*2),
                                 )


# #} end of get_config

# #{ save_pic

    def save_pic(self,req):
        rospy.loginfo('cur_img {}'.format(self.cur_img))
        rospy.loginfo('ing {}'.format(cv2.imwrite('/home/mrs/last_img.png',self.cur_img)))
        msg = HistMsg()
        msg.x, msg.y, msg.z = self.hsv_roi.shape
        msg.values = self.hsv_roi.flatten()

        buf_msg = BytesIO()
        buf_msg.seek(0)

        msg.serialize(buf_msg)
        f = open('/home/mrs/msg.bin', 'wb')
        f.write(buf_msg.getvalue())
        f.flush()

        return PicResponse()


# #} end of save_pic

# #{ freeze_callback

    def freeze_callback(self,req):
        if self.freeze:
            self.freeze = False
        else:
            self.freeze = True
        return FreezeResponse()


# #} end of freeze_callback

# #{ get_params_for_visualization

    def get_params(self, req):

        return ParamsResponse(self.config_path, self.save_dir, self.circled_name, self.circled_hsv_name, self.circled_lab_name,self.object_detect_name, self.save_to_drone)


# #} end of get_params_for_visualization

# #{ set_object_detect


    def set_object_detect(self, req):

        rospy.set_param(self.obd_h_c, float(self.h_mean))
        rospy.set_param(self.obd_h_r, float(self.h_sigma * self.sigma_multi_h*2))
        rospy.set_param(self.obd_s_c, float(self.s_mean))
        rospy.set_param(self.obd_s_r, float(self.s_sigma*self.sigma_multi_s*2))
        rospy.set_param(self.obd_v_c, float(self.v_mean))
        rospy.set_param(self.obd_v_s, float(self.v_sigma*self.sigma_multi_v*2))

        ## | --------------------- set HSV params --------------------- |

        rospy.set_param(self.obd_l_c, float(self.l_mean))
        rospy.set_param(self.obd_l_r, float(self.l_sigma*self.sigma_multi_l*2))
        rospy.set_param(self.obd_a_c, float(self.a_mean))
        rospy.set_param(self.obd_a_r, float(self.a_sigma*self.sigma_multi_a*2))
        rospy.set_param(self.obd_b_c, float(self.b_mean))
        rospy.set_param(self.obd_b_r, float(self.b_sigma*self.sigma_multi_b*2))
        rospy.set_param(self.obd_segment, req.color_space.data)
        rospy.set_param(self.ball_size, float(req.ball_rad.data))



        rospy.loginfo('params set')
        rospy.loginfo('h {} s {} v {}'.format(self.h_mean, self.s_mean, self.v_mean))
        rospy.loginfo('sigma h {} s {} v {}'.format(self.h_sigma, self.s_sigma, self.v_sigma))
        rospy.loginfo('l {} a {} b {}'.format(self.l_mean, self.a_mean, self.b_mean))
        rospy.loginfo('binarization name  {} '.format(req.color_space.data))
        resp = UpdateObdResponse()
        if self.object_update.call().success:
            resp.success = True
        else:
            resp.success = False

        return resp


# #} end of set_object_detect

# #{ change_callback

    def change_callback(self, req):
        self.sub = req.color_space
        resp = ChangeCallbackResponse()
        resp.success = True
        return resp



# #} end of change_callback

# #{ create_hist response

    def create_hist(self,req):
        mask = self.get_mask(req.x1, req.y1, req.x2, req.y2)  
        hsv = cv2.cvtColor(self.cur_img, cv2.COLOR_BGR2HSV)
        hbin = 180
        sbin = 256 
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
        if self.hist_hs is None:
            self.hist_hs = hist.astype('float')
        else:
            self.hist_hs += hist.astype('float')

        resp = CaptureHistResponse()
        resp.shape = hist.shape
        resp.hist = self.hist_hs.flatten().tolist()

        return resp




# #} end of create_hist response
       




if __name__ == '__main__':
    c = ColorCapture()
    rospy.spin()
    # menu()
