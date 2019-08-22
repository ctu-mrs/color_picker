from __future__ import division
import os
import subprocess
import rospy
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
    ParamsResponse

)
from PIL import Image
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton, QGroupBox, QRadioButton,QHBoxLayout
from python_qt_binding.QtGui import QPixmap, QImage
from argparse import ArgumentParser
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImg


HSV = 0
LUV = 1
RGB = 2
BOTH = 3

class ColorPlugin(Plugin):

    def __init__(self, context):
        super(ColorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ColorPlugin')
        rospy.sleep(1)
        print(context)
        self.brd = CvBridge()
        # Process standalone plugin command-line arguments
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self.view = RGB
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('balloon_color_picker'), 'resource', 'ColorPlugin.ui')
        loadUi(ui_file, self._widget)
        # self._widget.setStyleSheet("background-color:blue;")
        # Give QObjects reasonable names
        self._widget.setObjectName('ColorPluginUi')

        # print(rospy.get_param('gui_name')) 
        # ROS services 
 
        self.sigma_caller = rospy.ServiceProxy('change_sigma', ChangeSigma)
        self.sigma_lab_caller = rospy.ServiceProxy('change_sigma_lab', ChangeSigmaLab)
        self.caller = rospy.ServiceProxy('capture', Capture)
        self.get_count = rospy.ServiceProxy('get_count', GetCount)
        self.clear_count = rospy.ServiceProxy('clear_count', ClearCount)
        self.get_config = rospy.ServiceProxy('get_config', GetConfig)
        self.get_params = rospy.ServiceProxy('get_params', Params)


        rospy.wait_for_service('capture')
        rospy.wait_for_service('get_params')

        self.config_path, self.save_path, self.circled_param, self.circle_filter_param, self.circle_luv_param = self.set_params()
        # SUBS


        self.balloon_sub = rospy.Subscriber(self.circled_param, RosImg, self.img_callback, queue_size = 1)
        self.filter_sub  = rospy.Subscriber(self.circle_filter_param, RosImg, self.filter_callback, queue_size = 1)
        self.filter_luv  = rospy.Subscriber(self.circle_luv_param, RosImg, self.luv_callback, queue_size = 1)
        self.hsv = message_filters.Subscriber(self.circle_filter_param, RosImg)
        self.luv = message_filters.Subscriber(self.circle_luv_param, RosImg)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.luv,  self.hsv], 1, 0.5)
        self.ts.registerCallback(self.both_callback)
        
        self.colors = self.load_config(self.config_path)
        self.add_buttons(self.colors)
        context.add_widget(self._widget)


        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))




        # # DEFAULT IMAGE
        # img = cv2.imread('/home/mrs/balloon_workspace/src/ros_packages/balloon_color_picker/data/blue.png')
       
        # cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # h,w,c = img.shape
        # q_img = QImage(img.data, w,h,3*w, QImage.Format_RGB888)


        # q = QPixmap.fromImage(q_img)
        
        #DIRECTORY
        #default
        self._widget.directory.setText(self.save_path)
        self._widget.save_config.clicked.connect(self.save_config)


        #PLOT

        self.figure = Figure()
        self.figure_luv = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas_luv = FigureCanvas(self.figure_luv)
        self.canvas.setParent(self._widget.inner)
        # self.toolbar = NavigationToolbar(self.canvas,self._widget)
        # self.toolbar_luv = NavigationToolbar(self.canvas_luv,self._widget)
        # self.toolbar.setParent(self._widget.inner)
        # self.toolbar_luv.setParent(self._widget.inner_luv)
        self.canvas_luv.setParent(self._widget.inner_luv)
    
        #SLIDER CONFIG
        self._widget.sigma_slider.setRange(0,100)
        self._widget.sigma_slider.setSingleStep(1)
        self._widget.sigma_slider.setValue(6)


        self._widget.sigma_slider.valueChanged.connect(self.slider_event)

        

        self._widget.sigma_slider_s.setRange(0,100)
        self._widget.sigma_slider_s.setSingleStep(1)
        self._widget.sigma_slider_s.setValue(6)


        self._widget.sigma_slider_s.valueChanged.connect(self.slider_event)

        self._widget.sigma_slider_v.setRange(0,100)
        self._widget.sigma_slider_v.setSingleStep(1)
        self._widget.sigma_slider_v.setValue(6)


        self._widget.sigma_slider_v.valueChanged.connect(self.slider_event)

        self._widget.sigma_slider_lab.setRange(0,100)
        self._widget.sigma_slider_lab.setSingleStep(1)
        self._widget.sigma_slider_lab.setValue(6)


        self._widget.sigma_slider_lab.valueChanged.connect(self.slider_event_lab)

        self._widget.sigma_slider_a.setRange(0,100)
        self._widget.sigma_slider_a.setSingleStep(1)
        self._widget.sigma_slider_a.setValue(6)


        self._widget.sigma_slider_a.valueChanged.connect(self.slider_event_lab)


        self._widget.sigma_slider_b.setRange(0,100)
        self._widget.sigma_slider_b.setSingleStep(1)
        self._widget.sigma_slider_b.setValue(6)


        self._widget.sigma_slider_b.valueChanged.connect(self.slider_event_lab)



        #SIGMA TEXT
        font = self._widget.font()
        font.setPointSize(16)
        self._widget.sigma_value.setFont(font)
        self._widget.sigma_value_s.setFont(font)
        self._widget.sigma_value_v.setFont(font)
        self._widget.sigma_value_lab.setFont(font)
        self._widget.sigma_value_a.setFont(font)
        self._widget.sigma_value_b.setFont(font)
        #IMAGE COUNT TEXT
        self._widget.image_count.setFont(font)
        #BOX FOR BUTTONS font
        # self._widget.color_buttons.setFont(font)

        #LAB HSV TEXT
        font.setPointSize(23)
        self._widget.label_lab.setFont(font)
        self._widget.label_lab.hide()
        self._widget.label_hsv.setFont(font)
        self._widget.label_hsv.hide()

        # BUTTONS
        self._widget.change.clicked.connect(self.switch_view_hsv)
        self._widget.change_both.clicked.connect(self.switch_view_both)
        self._widget.change_luv.clicked.connect(self.switch_view_luv)
        self._widget.capture.clicked.connect(self.capture)
        self._widget.clear.clicked.connect(self.clear)
        # self._widget.wdg_img.setPixmap(q)
        # self._widget.box_layout.addWidget(self.toolbar)
        # self._widget.inner.box_layout.addWidget(self.canvas)
 
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

        self._widget.radio_buttons.setLayout(vbx)

        self.plotted = False



    def set_colorspace_hsv(self):
        self.color_space = 'HSV'

    def set_colorspace_lab(self):
        self.color_space = 'LAB'




    def plot(self, h,s,v,l,u,lv, means, sigmas):
        self.mean_h, self.mean_s, self.mean_v, self.mean_l, self.mean_u, self.mean_lv = means
        self.std_h, self.std_s, self.std_v, self.std_l, self.std_u, self.std_lv = sigmas

        self.figure.suptitle('HSV', fontsize=20)
        ax =self.figure.add_subplot(221)
        ax.clear()
        ax.hist(h, normed=True)
        ax.set_title('H', fontsize=16)
        ax.set_xlim([0,180])
        xmin, xmax = (0,180)
        x = np.linspace(xmin, xmax, 180)
        y = norm.pdf(x,self.mean_h,self.std_h)
        ax.plot(x, y)
        #thresholds
        val = float(self._widget.sigma_slider.value())/2

        ax.axvline(self.mean_h+self.std_h*val,color='r')
        ax.axvline(self.mean_h-self.std_h*val,color='g')

        sx =self.figure.add_subplot(222)
        sx.clear()
        sx.hist(s, normed=True)
        sx.set_title('S', fontsize=16)
        amin, xmax = (0,255)
        sx.set_xlim([0,255])
        x = np.linspace(xmin, xmax, 255)
        y = norm.pdf(x,self.mean_s,self.std_s)
        sx.plot(x, y)

        #thresholds
        val = float(self._widget.sigma_slider_s.value())/2

        sx.axvline(self.mean_s+self.std_s*val,color='r')
        sx.axvline(self.mean_s-self.std_s*val,color='g')

        vx =self.figure.add_subplot(223)
        vx.clear()
        vx.hist(v, normed=True)
        vx.set_title('V', fontsize=16)
        xmin, xmax = (0,255)
        vx.set_xlim([0,255])
        # vx.set_ylim([0,max(v)])
        x = np.linspace(xmin, xmax, 255)
        y = norm.pdf(x,self.mean_v,self.std_v)
        vx.plot(x, y)

        #thresholds
        val = float(self._widget.sigma_slider_v.value())/2

        vx.axvline(self.mean_v+self.std_v*val,color='r',ymax=1)
        vx.axvline(self.mean_v-self.std_v*val,color='g',ymax=1)

        # refresh canvas
        self.canvas.draw()

        self.figure_luv.suptitle('LAB', fontsize=20)
        ax =self.figure_luv.add_subplot(221)
        ax.clear()
        ax.set_title('L', fontsize=16)
        ax.hist(l, normed=True)
        xmin, xmax = (0,255)
        ax.set_xlim([0,255])
        x = np.linspace(xmin, xmax, 225)
        y = norm.pdf(x,self.mean_l,self.std_l)
        ax.plot(x, y)
        #thresholds
        val = float(self._widget.sigma_slider_lab.value())/2

        ax.axvline(self.mean_l+self.std_l*val,color='r')
        ax.axvline(self.mean_l-self.std_l*val,color='g')


        sx =self.figure_luv.add_subplot(222)
        sx.clear()
        sx.set_title('A', fontsize=16)
        sx.hist(u, normed=True)
        xmin, xmax = (0,256)
        x = np.linspace(xmin, xmax, 256)
        sx.set_xlim([0,256])
        y = norm.pdf(x,self.mean_u,self.std_u)
        sx.plot(x, y)
        #thresholds
        val = float(self._widget.sigma_slider_a.value())/2

        sx.axvline(self.mean_u+self.std_u*val,color='r')
        sx.axvline(self.mean_u-self.std_u*val,color='g')


        vx =self.figure_luv.add_subplot(223)
        vx.clear()
        vx.set_title('B', fontsize=16)
        vx.hist(lv, normed=True)
        xmin, xmax = (0,223)
        vx.set_xlim([0,223])
        x = np.linspace(xmin, xmax, 223)
        y = norm.pdf(x,self.mean_lv,self.std_lv)
        vx.plot(x, y)

        #thresholds
        val = float(self._widget.sigma_slider_b.value())/2

        vx.axvline(self.mean_lv+self.std_lv*val,color='r')
        vx.axvline(self.mean_lv-self.std_lv*val,color='g')


        # refresh canvas
        self.canvas_luv.draw()
        self.plotted = True

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
 



    def img_callback(self,data):
        if self.view != RGB:
            return
        img = self.brd.imgmsg_to_cv2(data, 'rgb8')
        # cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        h,w,c = img.shape
        q_img = QImage(img.data, w,h,3*w, QImage.Format_RGB888)


        q = QPixmap.fromImage(q_img)
        self._widget.wdg_img.setPixmap(q)

    def clear(self):
        self.figure.clf()
        self.clear_count()
        self._widget.image_count.setText('Samples taken: 0 ')
        print("cleared")

    def filter_callback(self,data):
        if self.view != HSV:
            return
        img = self.brd.imgmsg_to_cv2(data, 'rgb8')
        # cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        h,w,c = img.shape
        q_img = QImage(img.data, w,h,3*w, QImage.Format_RGB888)
        
        q = QPixmap.fromImage(q_img)
        self._widget.wdg_img.setPixmap(q)

    def luv_callback(self,data):
        if self.view != LUV:
            return
        img = self.brd.imgmsg_to_cv2(data, 'rgb8')
        # cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        h,w,c = img.shape
        q_img = QImage(img.data, w,h,3*w, QImage.Format_RGB888)


        q = QPixmap.fromImage(q_img)
        self._widget.wdg_img.setPixmap(q)

    def both_callback(self,luv,hsv):
        if self.view != BOTH:
            self._widget.label_hsv.hide()
            self._widget.label_lab.hide()
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
        self._widget.wdg_img.setPixmap(q)





    def slider_event(self):

        self.sigma_h = float(self._widget.sigma_slider.value())/2
        self.sigma_s = float(self._widget.sigma_slider_s.value())/2
        self.sigma_v = float(self._widget.sigma_slider_v.value())/2
        res = self.sigma_caller(self.sigma_h, self.sigma_s, self.sigma_v)

        self.update_plots()
    
        
        self._widget.sigma_value.setText('Sigma H value: {}'.format(self.sigma_h))
        self._widget.sigma_value_s.setText('Sigma S value: {}'.format(self.sigma_s))
        self._widget.sigma_value_v.setText('Sigma V value: {}'.format(self.sigma_v))

 
    def slider_event_lab(self):

        self.sigma_l = float(self._widget.sigma_slider_lab.value())/2
        self.sigma_a = float(self._widget.sigma_slider_a.value())/2
        self.sigma_b = float(self._widget.sigma_slider_b.value())/2
        self.update_plots_lab()
        # rospy.loginfo('value {}'.format(self.sigma_l))
        self.sigma_lab_caller(self.sigma_l, self.sigma_a, self.sigma_b)
        self._widget.sigma_value_lab.setText('Sigma L value: {}'.format(self.sigma_l))
        self._widget.sigma_value_a.setText('Sigma A value: {}'.format(self.sigma_a))
        self._widget.sigma_value_b.setText('Sigma B value: {}'.format(self.sigma_b))

     

    
    def capture(self):
    
        rospy.wait_for_service('capture')
        req = Capture()
        res= self.caller()
        # rospy.loginfo('response {}'.format(res))
        self.plot(res.h, res.s,res.v,res.l,res.u,res.lv, res.means, res.sigmas)
        self._widget.image_count.setText('Samples taken: {} '.format(res.count))
        return

        

    def switch_view_hsv(self):
        print("HSV")
        if self.view == HSV:
            self.view = RGB
            return
        self.view = HSV
        
    def switch_view_luv(self):
        print("LUV")
        if self.view == LUV:
            self.view = RGB
            return
        self.view = LUV

    def switch_view_both(self):
        print("BOTH")
        if self.view == BOTH:
            self.view = RGB

            return
        self.view = BOTH
               
        self._widget.label_hsv.show()
        self._widget.label_lab.show()

    def load_config(self, path):
        f = file(path,'r')
        print(path)
        res = yaml.safe_load(f)
        return res['colors'] 

    def clicked(self, color):

        def clicker():
            self._widget.directory.setText(self.save_path+'/{}.yaml'.format(color))
        return clicker

    def add_buttons(self, colors):
        vbx = QVBoxLayout()
        for color in colors:

            but = QPushButton('Color {}'.format(color))
            but.clicked.connect(self.clicked(color))
            vbx.addWidget(but)
        vbx.addStretch(1)
        self._widget.color_buttons.setLayout(vbx)

    def save_config(self):
        resp  = self.get_config()
        conf_obj = {}
        save_dir = self._widget.directory.text()
        name = save_dir.split('/')
        name = name[len(name)-1].split('.')[0]
        #HSV
        hsv = {}
        hsv['hue_center'] = resp.h[0]
        hsv['hue_range'] = resp.h[1]
        hsv['sat_center'] = resp.s[0]
        hsv['sat_range'] = resp.s[1]
        hsv['val_center'] = resp.v[0]
        hsv['val_range'] = resp.v[1]
        conf_obj['hsv'] = hsv
        #LAB 
        lab = {}
        lab['l_center'] = resp.l[0]
        lab['l_range'] = resp.l[1]
        lab['a_center'] = resp.a[0]
        lab['a_range'] = resp.a[1]
        lab['b_center'] = resp.b[0]
        lab['b_range'] = resp.b[1]
        conf_obj['lab'] = lab

        conf_obj['binarization_method'] = self.color_space
        if os.path.isdir(save_dir):
            return 
        f = file(save_dir,'w')
        path_to_script = os.path.join(rospkg.RosPack().get_path('balloon_color_picker'), 'scripts', 'copy_to_uav.sh')
        comm = '.'+path_to_script+' $UAV_NAME '+save_dir+' '+ name
        print('saved to dir {}'.format(save_dir))
        yaml.safe_dump(conf_obj,f)

        print('exectuted command ')
        print(path_to_script)
        print(save_dir)
        print(name)
        print(subprocess.check_call([path_to_script,'uav42', save_dir, name+'.yaml']))

    def set_params(self):
        
        resp = self.get_params()
        print(resp)
        print('params loaded')
        return resp.config_path, resp.save_path, resp.circled,resp.circle_filter, resp.circle_luv


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
