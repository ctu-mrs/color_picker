#!/usr/bin/env python
import sys
import rospy


from balloon_color_picker.plugin_module import ColorPlugin
from rqt_gui.main import Main


def main():

    plugin = 'balloon_color_picker'
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))


if __name__ == '__main__':
    main()
