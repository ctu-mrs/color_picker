
if [ $# -ne 3 ]; then
  echo Usage: ./copy_to_uav.sh UAV_NAME COLOR_PATH COLOR_FILENAME
  exit 1
fi

uav_name=$1
color_path=$2
color_fname=$3

scp $color_path/$color_fname mrs@$uav_name:/tmp/
if [ $? -ne 0 ]; then
  echo Could not copy color file to host! Ending
  exit 1
fi

uav_cmd='cp /tmp/'$color_fname' $(rospack find object_detect)/config/colors/'
ssh mrs@$uav_name "$uav_cmd"
if [ $? -ne 0 ]; then
  echo Could not move color file on host! Ending
  exit 1
fi

echo Successfully copied color to host.
