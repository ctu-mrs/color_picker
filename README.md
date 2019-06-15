# Color picker GUI 

## How to use

The basics setup is provided via launch/gui.launch file. You need to specify your image_in topic, the rest topics are for the gui, so you don't need to change them.
You need to supply also a path were you want the saved files to be - save_dir param.

Also, to make your life easier prepare a color config file, you can see an example in this git repo called balloon_config.yaml - it provides names of the colors, so you don't need to write the names by yourself, just click the button. For this needs there is a config_path param.

## To launch it type into your terminal: 
```
export $UAV_NAME=uav1; roslaunch balloon_color_picker gui.launch
```

