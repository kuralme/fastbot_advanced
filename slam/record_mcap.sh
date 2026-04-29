#!/bin/bash

ros2 bag record -s mcap --storage-config-file recording_config.yaml  \
  /tf \
  /tf_static \
  /fastbot/camera_odom \
  /fastbot/keyframes \
  /fastbot/keyframes_2d \
  /robot_description \
  /oak/left/image_rect/compressed \
  /oak/right/image_rect/compressed