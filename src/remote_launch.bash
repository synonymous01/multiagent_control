#!/usr/bin/env bash
ssh lums@robot1 "roslaunch realsense2_camera rs_camera tf_prefix:=robot1_camera"
ssh root1234@robot2 "roslaunch realsense2_camera rs_camera tf_prefix:=robot2_camera"
ssh robot4@robot3 "roslaunch realsense2_camera rs_camera tf_prefix:=robot3_camera"
