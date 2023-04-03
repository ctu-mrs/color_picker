from __future__ import division
# #{ imports
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
    ParamsResponse,
    Freeze,
    FreezeResponse,
)
from balloon_color_picker import MyWidget 

from PIL import Image
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton, QGroupBox, QRadioButton,QHBoxLayout, QShortcut
from python_qt_binding.QtGui import QPixmap, QImage, QKeySequence, QMouseEvent
from argparse import ArgumentParser
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImg



# #} end of imports


HSV = 0
LUV = 1
RGB = 2
BOTH = 3

class ColorPlugin(Plugin):

# #{ __init__

    def __init__(self, context):
        super(ColorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ColorPlugin')
        rospy.sleep(1)
        print(context)
        # Process standalone plugin command-line arguments
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        new_wdg = MyWidget.MyWidget()
        context.add_widget(new_wdg)


        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))


# #} end of __init__

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
