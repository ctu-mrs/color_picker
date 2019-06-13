#!/usr/bin/env python
import sys


from balloon_color_picker.plugin_module import ColorPlugin
from rqt_gui.main import Main

plugin = 'balloon_color_picker'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
