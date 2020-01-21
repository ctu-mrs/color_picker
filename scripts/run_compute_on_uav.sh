#!/bin/sh
if [ $# -ne 1 ]; then
  echo Usage: ./run_compute_on_uav.sh UAV_NAME
  exit 1
fi

uav_name=$1


uav_cmd='roslaunch balloon_color_picker compute.launch'
j
ssh mrs@$uav_name "$uav_cmd"
if [ $? -ne 0 ]; then
  echo Could not run the compute
  exit 1
fi

echo Successfully launched node on uav
