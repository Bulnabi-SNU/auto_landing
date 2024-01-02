<h1 align = "center"> PX4-Apriltag based auto landing control </h1>

## Demonstration & Processing pipeline description

<a href="https://youtube.com/shorts/I7M_TE6y5zg?feature=share"><p align="center">
    <img src="https://github.com/ArthurFDLR/drone-gesture-control/blob/main/.github/video_embed.PNG?raw=true" alt="Demonstration video" width="80%" style="border-radius: 5px;">
</p></a>

## Getting Started

### Step 1 - Prerequisites
Jetson Xavier NX (Jetpack --version--) - with PX4 board connected with GPIO pins.
@px4_msgs(submodule update).
@image_pipeline(submodule update).
ROS2 Foxy.
numpy.


### Step 2 - Configure Hardware

#### HOW TO ENABLE JETSON GPIO & CAMERA PIN FOR IMX477 CAMERA BOARD
  ```
  sudo /opt/nvidia/jetson-io/jetson-io.py
  ```

[HOW TO CONNECT JETSON-PX4(KOREAN)](https://chatter-thunder-3c7.notion.site/DDS-bc82970835b24221b3f8a057d0f3b49e?pvs=4).
### Step 3 - Install auto_landing package
  ```
    cd ~/ros2_ws/src/
    git clone https://github.com/One-star11/auto_landing.git
    cd ..
    colcon build --symlink-install
    source ~/ros2_ws/install/setup.bash
  ```
update your camera calibration by editing the code values in ./image_pub/src/mono_image_pub.cpp.
update your tag infos by editing the yaml file in ./apriltag_ros/share/apriltag_ros/cfg/tags_36h11.yaml.

### Step 4 - Start nodes
```
    #all in seperate terminals
    MicroXRCEAgent serial --dev /dev/ttyTHS0 -b 921600
    ros2 run image_pub mono_image_pub
    ros2 launch image_proc image_proc.launch.py
    ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_rect_color -r camera_info:=/camera/camera_info --params-file ros2 pkg prefix apriltag_ros/share/apriltag_ros/cfg/tags_36h11.yaml
    ros2 run goal_pub goal_pub
    ros2 run px4_offboard bezier_control
```


