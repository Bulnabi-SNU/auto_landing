<h1 align = "center"> PX4-Apriltag based auto landing control </h1>

## Demonstration

<p align="center">
  <img src="https://github.com/One-star11/auto_landing/assets/96758271/471160a7-8e76-4793-b352-9b751d802872">
</p>
</p></a>

## Getting Started
[NOTION Page](https://www.notion.so/APRILTAG-ebb209024fac4d3891294fd2af42cf43?pvs=4)
### Step 1 - Prerequisites
Jetson Xavier NX (Jetpack 5.0.2) - with PX4 board connected with GPIO pins   
-look for [HOW TO CONNECT JETSON-PX4] under  
ROS2 Foxy   
numpy  
FastDDS(MicroXRCEDDS)  


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
update your camera calibration by editing the code values in ./image_pub/src/mono_image_pub.cpp
update your tag infos by editing the yaml file in ./apriltag_ros/share/apriltag_ros/cfg/tags_36h11.yaml

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


