from __future__ import division
# #{ imports
import os
import subprocess
import rospy
import time
import rospkg
import cv2
import rospy
import message_filters
import random
import numpy as np
import yaml
from scipy.stats import norm
## matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

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
    GetConfig,
    GetConfigResponse,
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
from PIL import Image
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtCore import (
    QRect,
    Qt
)
from python_qt_binding.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QPushButton, 
    QGroupBox,
    QRadioButton,
    QHBoxLayout, 
    QShortcut,
    QRubberBand
)
from python_qt_binding.QtGui import (
    QPixmap,
    QImage,
    QKeySequence,
    QMouseEvent,
    QKeyEvent,
    QCursor
)
from std_msgs.msg import String
from argparse import ArgumentParser
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImg



# #} end of imports
# for setting the views HSV, LAB, RGB, or BOTH ( is kinda unused now)
HSV = 0
LUV = 1
RGB = 2
BOTH = 3
OBD = 4
# for cropping control
HIST = 20
IMG = 30
HIST_SELECTION = 22
HIST_DESELECTION = 25


class MyWidget(QWidget):

# #{ init

    def __init__(self):
        super(MyWidget, self).__init__()
        self.brd = CvBridge()
        self.view = RGB
        ui_file = os.path.join(rospkg.RosPack().get_path('balloon_color_picker'), 'resource', 'ColorPlugin.ui')
        rospy.loginfo('uifile {}'.format(ui_file))
        # self.log_info('uifile {}'.format(ui_file))
        loadUi(ui_file, self)
        # Give QObjects reasonable names
        self.setObjectName('ColorPluginUi')
        self.uav_name = os.environ['UAV_NAME']

        self.orig_h = 920
        self.orig_w = 1080
        self.hist_hsv_orig_h =  180
        self.hist_hsv_orig_w =  256
        self.hist_lab_orig_h =  256
        self.hist_lab_orig_w =  256
        self.hist_status = HSV
        self.select_status = HIST_SELECTION
        self.crop_stat = IMG
        self.hist_mask = np.zeros([self.hist_hsv_orig_h, self.hist_hsv_orig_w])
        self.hist_mask_lab = np.zeros([self.hist_lab_orig_h, self.hist_lab_orig_w])
        self.cur_hist_hs = None
        self.cur_hist_ab = None
        self.selected_count = 0
        # ROS services

# #{ ros services

        self.sigma_caller = rospy.ServiceProxy('change_sigma', ChangeSigma)
        self.sigma_lab_caller = rospy.ServiceProxy('change_sigma_lab', ChangeSigmaLab)
        self.caller = rospy.ServiceProxy('capture', Capture)
        self.capture_cropped_srv = rospy.ServiceProxy('capture_cropped', CaptureCropped)
        self.get_count = rospy.ServiceProxy('get_count', GetCount)
        self.clear_count = rospy.ServiceProxy('clear_count', ClearCount)
        self.get_config = rospy.ServiceProxy('get_config', GetConfig)
        self.get_params = rospy.ServiceProxy('get_params', Params)
        self.freeze_service = rospy.ServiceProxy('freeze', Freeze)
        self.update_service = rospy.ServiceProxy('change_obd', UpdateObd)
        self.change_callback = rospy.ServiceProxy('change_callback', ChangeCallback)
        self.capture_hist = rospy.ServiceProxy('capture_hist', CaptureHist)




# #} end of ros services        rospy.wait_for_service('capture')

        rospy.loginfo('waiting for service "get_params" from computation module \n if this is more than 5 secs check for topic remapping')
        # self.log_info('waiting for service')
        rospy.loginfo('uav_name {}'.format(os.environ['UAV_NAME']))
        # self.log_info('uav_name {}'.format(os.environ['UAV_NAME']))
        rospy.wait_for_service('get_params')

        self.config_path, self.save_path, self.circled_param, self.circle_filter_param, self.circle_luv_param, self.object_detect_param, self.save_to_drone = self.set_params()
        # SUBS
# #{ ros subs


        self.balloon_sub = rospy.Subscriber(self.circled_param, RosImg, self.img_callback, queue_size = 1)
        self.filter_sub  = rospy.Subscriber(self.circle_filter_param, RosImg, self.filter_callback, queue_size = 1)
        self.filter_luv  = rospy.Subscriber(self.circle_luv_param, RosImg, self.luv_callback, queue_size = 1)
        self.obj_det_sb = rospy.Subscriber(self.object_detect_param, RosImg, self.obj_det_callback, queue_size=1)
        # self.hsv = message_filters.Subscriber(self.circle_filter_param, RosImg)
        # self.luv = message_filters.Subscriber(self.circle_luv_param, RosImg)

        # self.ts = message_filters.ApproximateTimeSynchronizer([self.luv,  self.hsv], 1, 0.5)
        # self.ts.registerCallback(self.both_callback)


# #} end of ros subs

        self.colors = self.load_config(self.config_path)
        self.add_buttons(self.colors)





        # # DEFAULT IMAGE
        # img = cv2.imread('/home/mrs/balloon_workspace/src/ros_packages/balloon_color_picker/data/blue.png')

        # cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # h,w,c = img.shape
        # q_img = QImage(img.data, w,h,3*w, QImage.Format_RGB888)


        # q = QPixmap.fromImage(q_img)

        #DIRECTORY
        #default
        self.directory.setText(self.save_path+"Red.yaml")
        self.color_name = "red"
        self.save_button.clicked.connect(self.save_config)


        #PLOT

        self.figure = Figure()
        self.figure_luv = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas_luv = FigureCanvas(self.figure_luv)
        self.canvas.setParent(self.inner)
        # self.toolbar = NavigationToolbar(self.canvas,self)
        # self.toolbar_luv = NavigationToolbar(self.canvas_luv,self)
        # self.toolbar.setParent(self.inner)
        # self.toolbar_luv.setParent(self.inner_luv)
        self.canvas_luv.setParent(self.inner_luv)
        self.inner_luv.hide()
        self.inner_luv_hist.hide()

        #SLIDER CONFIG

        # #{ slider config
        
        self.sigma_slider.setRange(0,100)
        self.sigma_slider.setSingleStep(1)
        self.sigma_slider.setValue(6)


        self.sigma_slider.valueChanged.connect(self.slider_event)



        self.sigma_slider_s.setRange(0,100)
        self.sigma_slider_s.setSingleStep(1)
        self.sigma_slider_s.setValue(6)


        self.sigma_slider_s.valueChanged.connect(self.slider_event)

        self.sigma_slider_v.setRange(0,100)
        self.sigma_slider_v.setSingleStep(1)
        self.sigma_slider_v.setValue(80)


        self.sigma_slider_v.valueChanged.connect(self.slider_event)

        self.sigma_slider_lab.setRange(0,100)
        self.sigma_slider_lab.setSingleStep(1)
        self.sigma_slider_lab.setValue(80)


        self.sigma_slider_lab.valueChanged.connect(self.slider_event_lab)

        self.sigma_slider_a.setRange(0,100)
        self.sigma_slider_a.setSingleStep(1)
        self.sigma_slider_a.setValue(6)


        self.sigma_slider_a.valueChanged.connect(self.slider_event_lab)


        self.sigma_slider_b.setRange(0,100)
        self.sigma_slider_b.setSingleStep(1)
        self.sigma_slider_b.setValue(6)


        self.sigma_slider_b.valueChanged.connect(self.slider_event_lab)


        
        # #} end of slider config

# #{ font configs


        #SIGMA TEXT
        font = self.font()
        font.setPointSize(16)
        self.sigma_value.setFont(font)
        self.sigma_value_s.setFont(font)
        self.sigma_value_v.setFont(font)
        self.sigma_value_lab.setFont(font)
        self.sigma_value_a.setFont(font)
        self.sigma_value_b.setFont(font)
        #IMAGE COUNT TEXT
        self.image_count.setFont(font)
        #BOX FOR BUTTONS font
        # self.color_buttons.setFont(font)
        font.setPointSize(14)
        self.sigma_value.setFont(font)
        self.log_text.setFont(font)

        #LAB HSV TEXT
        font.setPointSize(23)
        self.label_lab.setFont(font)
        self.label_lab.hide()
        self.label_hsv.setFont(font)
        self.label_hsv.hide()


# #} end of font configs

        # BUTTONS
        self.change.clicked.connect(self.switch_view_hsv)
        self.change_both.clicked.connect(self.switch_view_both)
        self.change_luv.clicked.connect(self.switch_view_luv)
        self.change_object.clicked.connect(self.switch_view_object_detect)
        self.capture_button.clicked.connect(self.capture)
        self.clear_button.clicked.connect(self.clear)
        self.freeze_button.clicked.connect(self.freeze)
        self.update_button.clicked.connect(self.update_obd)
        # self.wdg_img.setPixmap(q)
        # self.box_layout.addWidget(self.toolbar)
        # self.inner.box_layout.addWidget(self.canvas)
        #shortcuts

        self.short_capture = QShortcut(QKeySequence("C"), self)
        self.short_capture.activated.connect(self.capture)
        self.short_hsv = QShortcut(QKeySequence("1"), self)
        self.short_hsv.activated.connect(self.switch_view_hsv)
        self.short_lab = QShortcut(QKeySequence("2"), self)
        self.short_lab.activated.connect(self.switch_view_luv)
        self.short_object_detect = QShortcut(QKeySequence("3"), self)
        self.short_object_detect .activated.connect(self.switch_view_object_detect)
        self.short_object_detect_update = QShortcut(QKeySequence("U"), self)
        self.short_object_detect_update .activated.connect(self.update_obd)
        self.short_both = QShortcut(QKeySequence("4"), self)
        self.short_both.activated.connect(self.switch_view_both)
        self.short_save = QShortcut(QKeySequence("S"), self)
        self.short_save.activated.connect(self.save_config)
        self.short_clear = QShortcut(QKeySequence("N"), self)
        self.short_clear.activated.connect(self.clear)
        self.short_freeze = QShortcut(QKeySequence("F"), self)
        self.short_freeze.activated.connect(self.freeze)

        vbx = QVBoxLayout()
        but_hsv = QRadioButton()
        but_hsv.setText('HSV')
        but_hsv.setChecked(True)
        self.color_space = 'HSV'
        but_hsv.clicked.connect(self.set_colorspace_hsv)
        vbx.addWidget(but_hsv)
        but_lab = QRadioButton()
        but_lab.setText('LAB')
        but_lab.clicked.connect(self.set_colorspace_lab)
        vbx.addWidget(but_lab)
        vbx.addStretch(1)

        self.radio_buttons.setLayout(vbx)
        vbx_method = QVBoxLayout()
        but_lut = QRadioButton()
        but_lut.setText('LUT')
        but_lut.setChecked(False)
        but_lut.clicked.connect(self.set_method_lut)
        vbx_method.addWidget(but_lut)

        but_thr = QRadioButton()
        but_thr.setText('Threshold')
        but_thr.setChecked(True)
        but_thr.clicked.connect(self.set_method_thr)
        vbx_method.addWidget(but_thr)
        vbx.addStretch(1)
        
        self.radio_buttons_method.setLayout(vbx_method)

        self.load_method = 'THR'

        # self.mousePressEvent.connect(self.mousePressEvent)
        self.plotted = False

        self._rubber = None

        self.frozen = False

# #} end of init

# #{ mouse events


    def mousePressEvent(self, QMouseEvent):
        cursor =QCursor()
        x = QMouseEvent.x()
        y = QMouseEvent.y()
        if (x < 1280 and y < 720) and ( x > 15 and y > 15):
            if self._rubber == None:
                if  not self.frozen:
                    self.freeze()
                    self.frozen_before = False
                else:
                    self.frozen_before = True
                self.rub_origin = QMouseEvent.pos()
                self._rubber = QRubberBand(QRubberBand.Rectangle, self)
                self._rubber.show()
                self.crop_stat  = IMG
        elif (x > 1300 and y > 520) and ( x < 1907 and  y < 1010  ):
            self.rub_origin = QMouseEvent.pos()
            self._rubber = QRubberBand(QRubberBand.Rectangle, self)
            self._rubber.show()
            if QMouseEvent.button() == Qt.RightButton and self.selected_count !=0:
                self.select_status = HIST_DESELECTION
            else:
                self.select_status = HIST_SELECTION
            self.crop_stat = HIST
            self.log_info("hist status {}".format("HSV" if self.hist_status == HSV else "LAB"))

 
    
    def mouseMoveEvent(self, QMouseEvent): 
        cursor =QCursor()
        x = QMouseEvent.x()
        y = QMouseEvent.y()
        # if in coords of image and crop status is image then draw the rect
        if self._rubber is None:
            return
        if (x < 1280 and y < 720) and ( x > 15 and y > 15) and self.crop_stat == IMG:
            self._rubber.setGeometry(QRect(self.rub_origin, QMouseEvent.pos()).normalized())
        # if in coords of hist and crop status is hist then draw the rect
        elif (x > 1300 and y > 520) and ( x < 1907 and  y < 1010  ) and self.crop_stat == HIST:
            self._rubber.setGeometry(QRect(self.rub_origin, QMouseEvent.pos()).normalized())


    def mouseReleaseEvent(self, QMouseEvent):
        cursor =QCursor()
        x = QMouseEvent.x()
        y = QMouseEvent.y()
        if self._rubber is None:
            return
        if (x < 1280 and y < 720) and ( x > 15 and y > 15) and self.crop_stat == IMG:

            if not self.frozen_before:
                self.freeze()

            a = self.mapToGlobal(self.rub_origin)
            b = QMouseEvent.globalPos()
            a = self.wdg_img.mapFromGlobal(a)
            b = self.wdg_img.mapFromGlobal(b)

            self._rubber.hide()
            self._rubber = None

            pix = QPixmap(self.wdg_img.pixmap())
            sx = float(self.wdg_img.rect().width())
            sy = float(self.wdg_img.rect().height())
            
            # h 1080 w 1920
            sx = self.orig_w / sx
            sy = self.orig_h / sy

            a.setX(int(a.x()*sx))
            a.setY(int(a.y()*sy))

            b.setX(int(b.x()*sx))
            b.setY(int(b.y()*sy))
            rect_ = QRect(a,b)

            h_ = rect_.height()
            w_ = rect_.width()

            y1,x1,y2,x2 = rect_.getCoords()
            rospy.loginfo('Img cropped x1 {} y1 {} x2 {} y2{}'.format(x1,y1,x2,y2))
            self.log_info('Img cropped x1 {} y1 {} x2 {} y2{}'.format(x1,y1,x2,y2))
            self.capture_cropped(x1,y1,x2,y2)
        elif (x > 1300 and y > 520) and ( x < 1907 and  y < 1010  ) and self.crop_stat == HIST:

            # h 1080 w 1920

            if self.hist_status == HSV:
                cur_hist = self.inner_hist
            elif self.hist_status == LUV:
                cur_hist = self.inner_luv_hist

           # if not self.frozen_before:
            #     self.freeze()
            a = self.mapToGlobal(self.rub_origin)
            b = QMouseEvent.globalPos()
            a = cur_hist.mapFromGlobal(a)
            b = cur_hist.mapFromGlobal(b)

            self._rubber.hide()
            self._rubber = None

            pix = QPixmap(cur_hist.pixmap())
            sx = float(cur_hist.rect().width())
            sy = float(cur_hist.rect().height())
            
            # h 1080 w 1920
            if self.hist_status == HSV:
                sx = self.hist_hsv_orig_w / sx
                sy = self.hist_hsv_orig_h / sy
            elif self.hist_status == LUV:
                sx = self.hist_lab_orig_w / sx
                sy = self.hist_lab_orig_h / sy

            a.setX(int(a.x()*sx))
            a.setY(int(a.y()*sy))

            b.setX(int(b.x()*sx))
            b.setY(int(b.y()*sy))
            rect_ = QRect(a,b)

            h_ = rect_.height()
            w_ = rect_.width()

            # y1,x1,y2,x2 = rect_.getCoords()
            x1,y1,x2,y2 = rect_.getCoords()
            rospy.loginfo('Hist cropped x1 {} y1 {} x2 {} y2 {}'.format(x1,y1,x2,y2))
            self.log_info('Hist cropped x1 {} y1 {} x2 {} y2 {}'.format(x1,y1,x2,y2))
            if self.select_status == HIST_SELECTION:
                self.select_hist(x1,y1,x2,y2, h_,w_, self.hist_status)
            elif self.select_status == HIST_DESELECTION:
                self.deselect_hist(x1,y1,x2,y2,self.hist_status)

        else:
            if self._rubber is not None:
                self._rubber.hide()
                self._rubber = None
                self.crop_stat = 0


# #} end of mouse events

# #{ set_colorspaces

    def set_colorspace_hsv(self):
        self.color_space = 'HSV'
        self.inner.show()
        self.inner_hist.show()
        self.hist_status = HSV
        self.log_info("hist status {}".format("HSV"))
        self.inner_luv.hide()
        self.inner_luv_hist.hide()
        if self.view == HSV:
            self.view = RGB
            self.set_view(self.view)
            return
        self.view = HSV
        self.log_info('HSV from radio button')
        self.set_view(self.view)

    def set_colorspace_lab(self):
        self.color_space = 'LAB'
        # but_lab.setChecked(True)
        self.inner_luv.show()
        self.inner_luv_hist.show()
        self.inner.hide()
        self.inner_hist.hide()
        self.hist_status = LUV
        self.log_info("hist status {}".format("LAB"))
        if self.view == LUV:
            self.view = RGB
            self.set_view(self.view)
            return
        rospy.loginfo('LAB from radio button')
        self.log_info('LAB from radio button')
        self.view = LUV
        self.set_view(self.view)

    def set_method_lut(self):
        self.load_method = 'LUT'
        self.log_info('Set method LUT')
        

    def set_method_thr(self):
        self.load_method = 'THR'
        self.log_info('Set method THR')


# #} end of set_colorspaces

# #{ plot

    def plot(self, h,s,v,l,u,lv, means, sigmas):
        self.mean_h, self.mean_s, self.mean_v, self.mean_l, self.mean_u, self.mean_lv = means
        self.std_h, self.std_s, self.std_v, self.std_l, self.std_u, self.std_lv = sigmas

        self.figure.suptitle('HSV', fontsize=20)
        ax =self.figure.add_subplot(221)
        ax.clear()
        ax.hist(h, density=True)
        ax.set_title('H', fontsize=16)
        ax.set_xlim([0,180])
        xmin, xmax = (0,180)
        x = np.linspace(xmin, xmax, 180)
        y = norm.pdf(x,self.mean_h,self.std_h)
        ax.plot(x, y)
        #thresholds
        val = float(self.sigma_slider.value())/2

        ax.axvline(self.mean_h+self.std_h*val,color='r')
        ax.axvline(self.mean_h-self.std_h*val,color='g')

        sx =self.figure.add_subplot(222)
        sx.clear()
        sx.hist(s, density=True)
        sx.set_title('S', fontsize=16)
        amin, xmax = (0,255)
        sx.set_xlim([0,255])
        x = np.linspace(xmin, xmax, 255)
        y = norm.pdf(x,self.mean_s,self.std_s)
        sx.plot(x, y)

        #thresholds
        val = float(self.sigma_slider_s.value())/2

        sx.axvline(self.mean_s+self.std_s*val,color='r')
        sx.axvline(self.mean_s-self.std_s*val,color='g')

        vx =self.figure.add_subplot(223)
        vx.clear()
        vx.hist(v, density=True)
        vx.set_title('V', fontsize=16)
        xmin, xmax = (0,255)
        vx.set_xlim([0,255])
        # vx.set_ylim([0,max(v)])
        x = np.linspace(xmin, xmax, 255)
        y = norm.pdf(x,self.mean_v,self.std_v)
        vx.plot(x, y)

        #thresholds
        val = float(self.sigma_slider_v.value())/2

        vx.axvline(self.mean_v+self.std_v*val,color='r',ymax=1)
        vx.axvline(self.mean_v-self.std_v*val,color='g',ymax=1)

        # refresh canvas
        self.canvas.draw()

        self.figure_luv.suptitle('LAB', fontsize=20)
        ax =self.figure_luv.add_subplot(221)
        ax.clear()
        ax.set_title('L', fontsize=16)
        ax.hist(l, density=True)
        xmin, xmax = (0,255)
        ax.set_xlim([0,255])
        x = np.linspace(xmin, xmax, 225)
        y = norm.pdf(x,self.mean_l,self.std_l)
        ax.plot(x, y)
        #thresholds
        val = float(self.sigma_slider_lab.value())/2

        ax.axvline(self.mean_l+self.std_l*val,color='r')
        ax.axvline(self.mean_l-self.std_l*val,color='g')


        sx =self.figure_luv.add_subplot(222)
        sx.clear()
        sx.set_title('A', fontsize=16)
        sx.hist(u, density=True)
        xmin, xmax = (0,256)
        x = np.linspace(xmin, xmax, 256)
        sx.set_xlim([0,256])
        y = norm.pdf(x,self.mean_u,self.std_u)
        sx.plot(x, y)
        #thresholds
        val = float(self.sigma_slider_a.value())/2

        sx.axvline(self.mean_u+self.std_u*val,color='r')
        sx.axvline(self.mean_u-self.std_u*val,color='g')


        vx =self.figure_luv.add_subplot(223)
        vx.clear()
        vx.set_title('B', fontsize=16)
        vx.hist(lv, density=True)
        xmin, xmax = (0,223)
        vx.set_xlim([0,223])
        x = np.linspace(xmin, xmax, 223)
        y = norm.pdf(x,self.mean_lv,self.std_lv)
        vx.plot(x, y)

        #thresholds
        val = float(self.sigma_slider_b.value())/2

        vx.axvline(self.mean_lv+self.std_lv*val,color='r')
        vx.axvline(self.mean_lv-self.std_lv*val,color='g')


        # refresh canvas
        self.canvas_luv.draw()
        self.plotted = True


# #} end of plot

# #{ update_plots

    def update_plots(self):
        ax = self.figure.get_axes()[0]
        sx = self.figure.get_axes()[1]
        vx = self.figure.get_axes()[2]
        # print(ax.lines)

        #thresholds
        del ax.lines[len(ax.lines)-1]
        del ax.lines[len(ax.lines)-1]

        up = self.mean_h+self.std_h*self.sigma_h
        if up > 180:
            up -=180
        down = self.mean_h-self.std_h*self.sigma_h
        if down < 0:
            down +=180
        ax.axvline(up,color='r')
        ax.axvline(down,color='g')

        #thresholds

        del sx.lines[len(sx.lines)-1]
        del sx.lines[len(sx.lines)-1]
        sx.axvline(self.mean_s+self.std_s*self.sigma_s,color='r')
        sx.axvline(self.mean_s-self.std_s*self.sigma_s,color='g')


        #thresholds

        del vx.lines[len(vx.lines)-1]
        del vx.lines[len(vx.lines)-1]
        vx.axvline(self.mean_v+self.std_v*self.sigma_v,color='r', ymax=1)
        vx.axvline(self.mean_v-self.std_v*self.sigma_v,color='g', ymax=1)

        self.canvas.draw()


# #} end of update_plots

# #{ update_plots_lab

    def update_plots_lab(self):
        # refresh canvas
        ax = self.figure_luv.get_axes()[0]
        sx = self.figure_luv.get_axes()[1]
        vx = self.figure_luv.get_axes()[2]
        # print(ax.lines)
        #thresholds
        del ax.lines[len(ax.lines)-1]
        del ax.lines[len(ax.lines)-1]

        ax.axvline(self.mean_l+self.std_l*self.sigma_l,color='r')
        ax.axvline(self.mean_l-self.std_l*self.sigma_l,color='g')


        #thresholds
        del sx.lines[len(sx.lines)-1]
        del sx.lines[len(sx.lines)-1]

        sx.axvline(self.mean_u+self.std_u*self.sigma_a,color='r')
        sx.axvline(self.mean_u-self.std_u*self.sigma_a,color='g')



        #thresholds
        del vx.lines[len(vx.lines)-1]
        del vx.lines[len(vx.lines)-1]

        vx.axvline(self.mean_lv+self.std_lv*self.sigma_b,color='r')
        vx.axvline(self.mean_lv-self.std_lv*self.sigma_b,color='g')


        # refresh canvas
        self.canvas_luv.draw()



# #} end of update_plots_lab

# #{ img_callback

    def img_callback(self,data):
        if self.view != RGB:
            return
        img = self.brd.imgmsg_to_cv2(data, 'rgb8')
        h,w,c = img.shape
        self.orig_h = h
        self.orig_w = w
        # rospy.loginfo('h {} w {}'.format(h,w))
        # cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = cv2.resize(img, dsize=(1280,720), interpolation=cv2.INTER_CUBIC)
        h,w,c = img.shape
        q_img = QImage(img.data, w,h,3*w, QImage.Format_RGB888)


        q = QPixmap.fromImage(q_img)
        self.wdg_img.setFixedWidth(1280)
        self.wdg_img.setFixedHeight(720)
        self.wdg_img.setPixmap(q)


# #} end of img_callback

# #{ clear

    def clear(self):
        self.figure.clf()
        self.clear_count()
        self.image_count.setText('Samples: 0 ')
        print("cleared")
        self.log_info("cleared")


# #} end of clear

# #{ filter callback

    def filter_callback(self,data):
        if self.view != HSV:
            return
        img = self.brd.imgmsg_to_cv2(data, 'rgb8')
        # cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = cv2.resize(img, dsize=(1280,720), interpolation=cv2.INTER_CUBIC)
        h,w,c = img.shape

        q_img = QImage(img.data, w,h,3*w, QImage.Format_RGB888)

        q = QPixmap.fromImage(q_img)

        self.wdg_img.setFixedWidth(1280)
        self.wdg_img.setFixedHeight(720)
        self.wdg_img.setPixmap(q)

# #} end of filter callback

# #{ luv_callback

    def luv_callback(self,data):
        if self.view != LUV:
            return
        img = self.brd.imgmsg_to_cv2(data, 'rgb8')
        # cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = cv2.resize(img, dsize=(1280,720), interpolation=cv2.INTER_CUBIC)
        h,w,c = img.shape
        # rospy.loginfo('shape {}'.format(img.shape))

        q_img = QImage(img.data, w,h,3*w, QImage.Format_RGB888)


        q = QPixmap.fromImage(q_img)
        # self.wdg_img.setFixedWidth(w)
        # self.wdg_img.setFixedHeight(h)

        self.wdg_img.setPixmap(q)



# #} end of luv_callback

# #{ object_detect_callback

    def obj_det_callback(self,data):
        if self.view != OBD:
            return
        if self.frozen:
            return
        img = self.brd.imgmsg_to_cv2(data, 'rgb8')
        # cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = cv2.resize(img, dsize=(1280,720), interpolation=cv2.INTER_CUBIC)
        h,w,c = img.shape
        q_img = QImage(img.data, w,h,3*w, QImage.Format_RGB888)


        q = QPixmap.fromImage(q_img)
        self.wdg_img.setFixedWidth(w)
        self.wdg_img.setFixedHeight(h)
        self.wdg_img.setPixmap(q)


# #} end of luv_callback

# #{ both_callback

    def both_callback(self,luv,hsv):
        if self.view != BOTH:
            self.label_hsv.hide()
            self.label_lab.hide()
            return
        img_luv = self.brd.imgmsg_to_cv2(luv)
        img_hsv = self.brd.imgmsg_to_cv2(hsv)
        # cv2.cvtColor(img_luv, cv2.COLOR_BGR2RGB)
        # cv2.cvtColor(img_hsv, cv2.COLOR_BGR2RGB)

        h,w,c = img_luv.shape
        img = np.zeros([h,w,c])
        luv_2 = cv2.resize(img_luv, (0,0), fx=0.5, fy=0.5)
        hsv_2 = cv2.resize(img_hsv, (0,0), fx=0.5, fy=0.5)
        both = np.hstack((hsv_2,luv_2))
        dif = (img.shape[0] - both.shape[0])//2
        img[dif:img.shape[0]-dif,0:img.shape[1]] = both
        q_img = QImage(both.data, both.shape[1],both.shape[0],3*both.shape[1], QImage.Format_RGB888)


        q = QPixmap.fromImage(q_img)
        self.wdg_img.setFixedWidth(both.shape[1])
        self.wdg_img.setFixedHeight(both.shape[0])
        self.wdg_img.setPixmap(q)



# #} end of both_callback

# #{ slider_event_hsv

    def slider_event(self):

        self.sigma_h = float(self.sigma_slider.value())/2
        self.sigma_s = float(self.sigma_slider_s.value())/2
        self.sigma_v = float(self.sigma_slider_v.value())/2
        res = self.sigma_caller(self.sigma_h, self.sigma_s, self.sigma_v)

        if len(self.figure.get_axes()) > 0:
            rospy.loginfo('axes {}'.format(self.figure.get_axes()))
            self.log_info('axes {}'.format(self.figure.get_axes()))
            self.update_plots()


        self.sigma_value.setText('Sigma H value: {}'.format(self.sigma_h))
        self.sigma_value_s.setText('Sigma S value: {}'.format(self.sigma_s))
        self.sigma_value_v.setText('Sigma V value: {}'.format(self.sigma_v))


# #} end of slider_event_hsv

# #{ slider_event_lab

    def slider_event_lab(self):

        self.sigma_l = float(self.sigma_slider_lab.value())/2
        self.sigma_a = float(self.sigma_slider_a.value())/2
        self.sigma_b = float(self.sigma_slider_b.value())/2
        if len(self.figure_luv.get_axes()) > 0:
            self.update_plots_lab()
        # rospy.loginfo('value {}'.format(self.sigma_l))
        # self.log_info('value {}'.format(self.sigma_l))
        self.sigma_lab_caller(self.sigma_l, self.sigma_a, self.sigma_b)
        self.sigma_value_lab.setText('Sigma L value: {}'.format(self.sigma_l))
        self.sigma_value_a.setText('Sigma A value: {}'.format(self.sigma_a))
        self.sigma_value_b.setText('Sigma B value: {}'.format(self.sigma_b))



# #} end of slider_event_lab

# #{ capture

    def capture(self):

        rospy.wait_for_service('capture')
        req = Capture()
        res= self.caller()
        # rospy.loginfo('response {}'.format(res))
        # self.log_info('response {}'.format(res))
        self.plot(res.h, res.s,res.v,res.l,res.u,res.lv, res.means, res.sigmas)
        self.image_count.setText('Samples taken: {} '.format(res.count))
        return


# #} end of capture

# #{ capture_cropped

    def capture_cropped(self, x1,y1,x2,y2):
        t = time.time()

        res  =  self.capture_cropped_srv(x1,y1,x2,y2)
        rospy.loginfo('time for capture cropped {}'.format(time.time() - t))
        
        t = time.time()
        hist =  self.capture_hist(x1,y1,x2,y2)
        
        self.set_hist(hist)
        rospy.loginfo('time for hist cropped {}'.format(time.time() - t))
        if res.success == False:
            rospy.loginfo('capture cropped returned false, NaNs are possible')
            return

        # rospy.loginfo('response {}'.format(res))

        t = time.time()
        self.plot(res.h, res.s,res.v,res.l,res.u,res.lv, res.means, res.sigmas)

        rospy.loginfo('time for plot {}'.format(time.time() - t))
        self.image_count.setText('Samples: {} '.format(res.count))



# #} end of capture_cropped

# #{ switch_view_hsv

    def switch_view_hsv(self):
        if self.view == HSV:
            self.view = RGB
            self.set_view(self.view)
            return
        print("HSV")
        self.log_info("HSV")
        self.hist_status = HSV
        self.view = HSV
        # rospy.loginfo('HSV from radio button {}'.format(self.radio_buttons.buttonClicked()))
        self.set_view(self.view)
        self.inner.show()
        self.inner_hist.show()
        self.inner_luv.hide()
        self.inner_luv_hist.hide()



# #} end of switch_view_hsv

# #{ switch_view_luv

    def switch_view_luv(self):
        if self.view == LUV:
            self.view = RGB
            self.set_view(self.view)
            return
        print("LUV")
        self.hist_status = LUV
        self.log_info("LUV")
        self.view = LUV
        self.set_view(self.view)
        self.inner_luv.show()
        self.inner_luv_hist.show()
        self.inner.hide()
        self.inner_hist.hide()



# #} end of switch_view_luv

# #{ update object detect colors

    def update_obd(self):
        rospy.loginfo('Sending data to Object Detect:{}'.format(self.load_method))
        self.log_info('Sending data to Object Detect:{}'.format(self.load_method))
        ball_rad = String()
        ball_rad.data = self.ball_radius.text()
        if self.hist_status == HSV:
            hist = np.transpose(self.hist_mask).flatten().astype('uint8')
            shape = self.hist_mask.shape
        elif self.hist_status == LUV:
            hist = np.transpose(self.hist_mask_lab).flatten().astype('uint8')
            shape = self.hist_mask_lab.shape

        if self.ball_radius.text() == "":
            return
        method = String()
        method.data = self.load_method

        color = String()
        if self.load_method == 'LUT':
            rospy.loginfo('load method YES {}'.format(self.load_method))
            rospy.loginfo('color space {}'.format(self.color_space))
            if self.color_space == 'HSV':
                color.data = 'hs_lut'
            elif self.color_space == 'LAB':
                rospy.loginfo('Color space {}'.format(self.color_space))
                color.data = 'ab_lut'
        else:
            rospy.loginfo('load method is {}'.format(self.load_method))
            color.data = self.color_space
        rospy.loginfo('color {}'.format(color))

        try:
            rospy.loginfo('updating object detect {}'.format(self.update_service.call(color,ball_rad,method, hist, shape)))
            self.log_info('updating object detect {}'.format("OBD updated"))
        except:
            rospy.loginfo("Couldn't update the object detect")
            self.log_info("Couldn't update the object detect")

        if self.frozen:
            self.freeze()



# #} end of update object detect colors

# #{ switch_view_both

    def switch_view_both(self):
        if self.view == BOTH:
            self.view = RGB
            self.set_view(self.view)
            return
        print("BOTH")
        self.log_info("BOTH")
        self.view = BOTH
        self.set_view(self.view)

        self.label_hsv.show()
        self.label_lab.show()


# #} end of switch_view_both

# #{ switch_view_object_detect

    def switch_view_object_detect(self):
        if self.view == OBD:
            self.view = RGB
            return
        print("OBD")
        self.log_info("OBD")
        self.view = OBD

# #} end of switch_view_both

# #{ load_config

    def load_config(self, path):
        # rospy.loginfo('cur dir {}'.format(os.path.curdir))
        # path = os.path.join(os.path.curdir,'../../config/balloon_config.yaml')
        # f = file(path,'r')
        # print(path)
        # res = yaml.safe_load(f)
        # TODO: change to reading from a file
        colors = ['Red','Green', 'Blue', 'Yellow']
        return colors


# #} end of load_config

# #{ clicked

    def clicked(self, color):

        def clicker():
            self.color_name = color
            self.directory.setText(self.save_path+'/{}.yaml'.format(color))
        return clicker


# #} end of clicked

# #{ add_buttons

    def add_buttons(self, colors):
        vbx = QVBoxLayout()
        i = 1
        for color in colors:

            but = QPushButton('Color {} ({})'.format(color, i))
            but.clicked.connect(self.clicked(color))

            but_short = QShortcut(QKeySequence("Ctrl+"+str(i)), self)
            but_short.activated.connect(self.clicked(color))
            i+=1
            vbx.addWidget(but)

        vbx.addStretch(1)
        self.color_buttons.setLayout(vbx)


# #} end of add_buttons

# #{ freeze

    def freeze(self):
        self.freeze_service.call()
        if self.frozen:
            self.frozen = False
        else:
            self.frozen = True
        rospy.loginfo('Frozen {}'.format(self.frozen))
        self.log_info('Frozen {}'.format(self.frozen))
        return



# #} end of freeze

# #{ save_config

    def save_config(self):
        color = String()
        color.data = self.color_space
        save_dir = String()
        save_dir.data = self.directory.text()
        ball_rad = String()
        if self.ball_radius.text() == "":
            return
        if os.path.isdir(self.directory.text()):
            return
        ball_rad.data = self.ball_radius.text()
        color_name = String()
        rospy.loginfo('Saving with load method {}'.format(self.load_method))
        hist = None
        hist_shape = None
        if self.load_method == 'LUT':
            self.log_info('Saving lut type with color space : {}'.format(self.color_space))
            if self.color_space == 'HSV':
                rospy.loginfo('hs_lut')
                color.data = 'hs_lut'
                hist_shape = self.hist_mask.shape
                hist = np.transpose(self.hist_mask).flatten().astype('uint8')
            elif self.color_space == 'LAB':
                rospy.loginfo('ab_lut')
                color.data = 'ab_lut'
                hist = np.transpose(self.hist_mask_lab).flatten().astype('uint8')
                hist_shape = self.hist_mask_lab.shape
        else:
            color_name.data = self.color_name
        rospy.loginfo('color space to SAVE {}'.format(color_name))

        resp  = self.get_config(color, save_dir, ball_rad, color_name, hist, hist_shape)

        #conf_obj = {}
        ##HSV
        #hsv = {}
        ## hsv['hist_bins_h'] = resp.hsv[0].bins
        ## hsv['hist_hist_h'] = resp.hsv[0].values
        ## hsv['hist_bins_s'] = resp.hsv[1].bins
        ## hsv['hist_hist_s'] = resp.hsv[1].values
        ## hsv['hist_bins_v'] = resp.hsv[2].bins
        ## hsv['hist_hist_v'] = resp.hsv[2].values
        ## hsv['hsv_roi'] = resp.hsv_roi
        #hsv['hue_center'] = resp.h[0]
        #hsv['hue_range'] = resp.h[1]
        #hsv['sat_center'] = resp.s[0]
        #hsv['sat_range'] = resp.s[1]
        #hsv['val_center'] = resp.v[0]
        #hsv['val_range'] = resp.v[1]
        #conf_obj['hsv'] = hsv
        ##LAB
        #lab = {}
        #lab['l_center'] = resp.l[0]
        #lab['l_range'] = resp.l[1]
        #lab['a_center'] = resp.a[0]
        #lab['a_range'] = resp.a[1]
        #lab['b_center'] = resp.b[0]
        #lab['b_range'] = resp.b[1]
        ## lab['hist_bins_l'] = resp.lab[0].bins
        ## lab['hist_hist_l'] = resp.lab[0].values
        ## lab['hist_bins_a'] = resp.lab[1].bins
        ## lab['hist_hist_a'] = resp.lab[1].values
        ## lab['hist_bins_b'] = resp.lab[2].bins
        ## lab['hist_hist_b'] = resp.lab[2].values
        ## lab['lab_roi'] = resp.lab_roi
        #conf_obj['lab'] = lab

        #conf_obj['binarization_method'] = self.color_space
        #if os.path.isdir(save_dir):
        #    return
        #f = file(save_dir,'w')
        #print('saved to dir {}'.format(save_dir))
        #yaml.safe_dump(conf_obj,f)

        #if self.save_to_drone:
        #    path_to_script = os.path.join(rospkg.RosPack().get_path('balloon_color_picker'), 'scripts', 'copy_to_uav.sh')
        #    print(path_to_script)
        #    print('exectuted command ')
        #    print(save_dir)
        #    print(subprocess.check_call([path_to_script,os.environ['UAV_NAME'], save_dir, name+'.yaml']))


# #} end of save_config

# #{ set_params

    def set_params(self):

        resp = self.get_params()
        print(resp)
        print('params loaded')
        return resp.config_path, resp.save_path, resp.circled,resp.circle_filter, resp.circle_luv, resp.object_detect, resp.save_to_drone

# #} end of set_params

# #{ set_view

    def set_view(self, view):
        self.change_callback(view)



# #} end of set_view        

# #{ set_hist

    def set_hist(self, hist_resp):
        hist = np.array(hist_resp.hist_hsv)
        hist = np.reshape(hist, hist_resp.shape_hsv)
        self.selected_count = 0

        self.hist_hsv_orig_h = hist_resp.shape_hsv[0]
        self.hist_hsv_orig_w = hist_resp.shape_hsv[1]
        self.cur_hist_hs = hist
        # self.hist_mask = np.zeros(self.cur_hist_hs.shape)

        hist = np.array(hist_resp.hist_lab)
        hist = np.reshape(hist, hist_resp.shape_lab)


        self.hist_lab_orig_h = hist_resp.shape_lab[0]
        self.hist_lab_orig_w = hist_resp.shape_lab[1]
        self.cur_hist_ab = hist


        self.redraw(HSV)
        self.redraw(LUV)

# #} end of set_hist

# #{ draw_hist


    def draw_hist(self, histRGB):

        # histRGB = np.log2(histRGB)
        new_h = cv2.resize(histRGB.astype('uint8'), dsize=(512,360), interpolation=cv2.INTER_CUBIC)
        # new_h = histRGB.copy().astype('uint8')

        # cv2.imshow("to draw", new_h)
        # cv2.waitKey(1)
        rospy.loginfo('new_h shape {}'.format(new_h.shape))

        h,w,c = new_h.shape
        total = new_h.nbytes
        perLine = int(total/h)
        if c == 3:
            q_img = QImage(new_h.data, w,h,perLine, QImage.Format_RGB888)
        elif c ==4:
            q_img = QImage(new_h.data, w,h,perLine, QImage.Format_RGBA8888)

        q = QPixmap.fromImage(q_img)
        self.inner_hist.setFixedWidth(512)
        self.inner_hist.setFixedHeight(360)
        self.inner_hist.setPixmap(q)


# #} end of draw_hist

# #{ draw_hist_lab


    def draw_hist_lab(self, histRGB):

        # histRGB = np.log2(histRGB)
        new_h = cv2.resize(histRGB.astype('uint8'), dsize=(400,400), interpolation=cv2.INTER_CUBIC)
        # new_h = histRGB.copy().astype('uint8')

        # cv2.imshow("to draw", new_h)
        # cv2.waitKey(1)
        rospy.loginfo('new_h shape {}'.format(new_h.shape))

        h,w,c = new_h.shape
        total = new_h.nbytes
        perLine = int(total/h)
        if c == 3:
            q_img = QImage(new_h.data, w,h,perLine, QImage.Format_RGB888)
        elif c ==4:
            q_img = QImage(new_h.data, w,h,perLine, QImage.Format_RGBA8888)

        q = QPixmap.fromImage(q_img)
        self.inner_luv_hist.setFixedWidth(400)
        self.inner_luv_hist.setFixedHeight(400)
        self.inner_luv_hist.setPixmap(q)


# #} end of draw_hist

# #{ select_hist

    def select_hist(self, x1,y1,x2,y2, h, w, color_space):
        rospy.loginfo('select status {}'.format(self.select_status))
        self.selected_count += 1
        if color_space == HSV:
            cv2.rectangle(self.hist_mask, (x1, y1), (x2, y2), (1), -1) 
        elif color_space == LUV:
            cv2.rectangle(self.hist_mask_lab, (x1, y1), (x2, y2), (1), -1)
        self.redraw(color_space)

    def redraw(self, color_space):
        if color_space == HSV:
            if self.cur_hist_hs is None:
                return
            hist = self.cur_hist_hs.copy()
        elif color_space == LUV:
            if self.cur_hist_ab is None:
                return
            hist = self.cur_hist_ab.copy()

        #normalizing the histogram
        minVal, maxVal, l, m = cv2.minMaxLoc(hist)
        hist = (hist-minVal)/(maxVal-minVal)*255.0

        rospy.loginfo('hist shape {}'.format(hist.shape))

        hist = cv2.equalizeHist(hist.astype('uint8'))
        histRGB = cv2.cvtColor(hist.astype('uint8'), cv2.COLOR_GRAY2RGB)


        if color_space == HSV:
            maskRGB = cv2.cvtColor(self.hist_mask.astype('uint8'), cv2.COLOR_GRAY2RGB)

            maskRGB[self.hist_mask>0, :] = np.array([255,0,0])
        elif color_space == LUV:
            maskRGB = cv2.cvtColor(self.hist_mask_lab.astype('uint8'), cv2.COLOR_GRAY2RGB)
            maskRGB[self.hist_mask_lab>0, :] = np.array([255,0,0])


        alpha = 0.3        
        selected_hist = alpha*maskRGB + (1.0-alpha)*histRGB
        rospy.loginfo('to draw shape {}'.format(selected_hist.shape))
            

        if color_space == HSV:
            self.draw_hist(selected_hist) 
        elif color_space == LUV:
            self.draw_hist_lab(selected_hist)



# #} end of select_hist

# #{ deselect_hist

    def deselect_hist(self, x1,y1,x2,y2, color_space):
        rospy.loginfo('deselect')
        self.log_info('deselect')
        if self.selected_count == 0:
            rospy.loginfo('nothing is selected, can"t deselect')
            self.log_info('nothing is selected, can"t deselect')
            return
        if color_space == HSV:
            cv2.rectangle(self.hist_mask, (x1, y1), (x2, y2), (0), -1)  # A filled rectangle
        elif color_space == LUV:
            cv2.rectangle(self.hist_mask_lab, (x1, y1), (x2, y2), (0), -1)  # A filled rectangle
        self.redraw(color_space)


# #} end of deselect_hist

# #{ get_mask

    def get_mask(self,arr, x1,y1,x2,y2):
        mask = np.zeros([arr.shape[0], arr.shape[1]])
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

# #{ keyPressEvent
    def keyPressEvent(self,event):
        pass


# #} end of keyPressEvent

# #{ default todo's

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog


# #} end of default todo's

    def log_info(self, text):
        self.log_text.setText("Last log message: {}".format(text))



