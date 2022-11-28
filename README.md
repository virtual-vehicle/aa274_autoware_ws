# Autoware "Hands-On"

Stanford Lecture AA274 / Graz University of Technology 


![Autonomous Racing Graz / Localization](https://github.com/virtual-vehicle/aa274_autoware_ws/blob/master/docs/ARG_Localization.jpg?raw=true "Autonomous Racing Graz / Localiztion")



[M. Schratter, J. Zubaca, K. Mautner-Lassnig, T. Renzler, M. Kirchengast, S. Loigge, M. Stolz and D. Watzenig, Lidar-based Mapping and Localization for Autonomous Racing, Opportunities and Challenges with Autonomous Racing, ICRA, 2021.](https://linklab-uva.github.io/icra-autonomous-racing/contributed_papers/paper4.pdf)

## Preconditions
- Ubuntu 20.04 (64 bit) is installed --> http://releases.ubuntu.com/20.04/
- ROS (noetix) is installed --> http://wiki.ros.org/noetic/Installation/Ubuntu

Installation of required packages
```
sudo apt update
sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin python-catkin-tools
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
sudo apt install -y git-lfs
pip3 install -U setuptools
```


# Install workspace for AA274 lecture
Clone repository
```
git clone https://github.com/virtual-vehicle/aa274_autoware_ws.git ~/ros/aa274_autoware_ws
git clone https://github.com/autonomousracing-ai/arg_demos ~/ros/aa274_autoware_ws/src/arg_demos
git clone https://github.com/autonomousracing-ai/arg_devbot_description ~/ros/aa274_autoware_ws/src/arg_devbot_description
git clone https://github.com/autonomousracing-ai/arg_data_croix_en_ternois ~/ros/aa274_autoware_ws/src/arg_data_croix_en_ternois
git clone https://github.com/autonomousracing-ai/arg_localization ~/ros/aa274_autoware_ws/src/arg_localization 

cd ~/ros/aa274_autoware_ws/src/arg_data_croix_en_ternois/bagfile
tar -xvf devbot_lap0.tar.xz
```
Setup for environment which includes several aliases, environment variables, and sources 
```
echo "export AA274_WS_DIR=~/ros/aa274_autoware_ws" >> ~/.bashrc
echo "source ~/ros/aa274_autoware_ws/environment.sh" >> ~/.bashrc
```	
**Build the workspace**

Close and open a new terminal to source the Autoware.AI components!

```
source /opt/ros/noetic/setup.bash
cd ~/ros/aa274_autoware_ws/
catkin_make
```
Extract recorded data from the racetrack
```
cd ~/ros/aa274_autoware_ws/src/arg_data_croix_en_ternois/bagfile
tar -xvf devbot_lap0.tar.xz
```



# Demo Localization
### Terminal 1:
```
roscore 
```

### Terminal 2:
Play a rosbag
```
roslaunch arg_data_croix_en_ternois play_rosbag.launch 
```

### Terminal 3:
GPS-based localization
```
roslaunch arg_demos arg_demo_lidar_localization.launch
```
or Lidar-based localization
```
roslaunch arg_demos arg_demo_lidar_localization.launch lidar_localization:=true
```

You must hit the **space key** to start to play the rosbag.

### Possible modifications can be performed:
- [Adding noise to the GPS data](https://github.com/autonomousracing-ai/arg_localization/blob/main/arg_localization/launch/gps_pose_estimator.launch#L4)
- [Activate/deactivate the input pose for the EKF](https://github.com/autonomousracing-ai/arg_demos/blob/main/launch/arg_demo_localization.launch#L81)
- [Activate/deactivate the input twist for the EKF](https://github.com/autonomousracing-ai/arg_demos/blob/main/launch/arg_demo_localization.launch#L88)
- [Modify the standard deviation parameters for the input pose](https://github.com/autonomousracing-ai/arg_demos/blob/main/launch/arg_demo_localization.launch#L83)



