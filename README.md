# Autoware "Hands-On"

Stanford Lecture AA274 / Graz University of Technology 


![Autonomous Racing Graz / Localization](https://github.com/virtual-vehicle/aa274_autoware_ws/blob/master/docs/ARG_Localization.jpg?raw=true "Autonomous Racing Graz / Localiztion")



[M. Schratter, J. Zubaca, K. Mautner-Lassnig, T. Renzler, M. Kirchengast, S. Loigge, M. Stolz and D. Watzenig, Lidar-based Mapping and Localization for Autonomous Racing, Opportunities and Challenges with Autonomous Racing, ICRA, 2021.](https://linklab-uva.github.io/icra-autonomous-racing/contributed_papers/paper4.pdf)

## Preconditions
- Ubuntu 18.04 (64 bit) is installed --> http://releases.ubuntu.com/18.04/
- ROS (melodic) is installed --> http://wiki.ros.org/melodic/Installation/Ubuntu

# Install Autoware.AI by source
(https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/Source-Build)

Installation of required packages
```
sudo apt update
sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin python-catkin-tools
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
sudo apt install ros-$ROS_DISTRO-lanelet2-*
sudo apt install -y git-lfs
pip3 install -U setuptools
```
Create workspace for Autoware.AI
```
mkdir -p ~/ros/autoware.ai/src
cd ~/ros/autoware.ai
```
Download the workspace configuration for Autoware.AI
```
wget -O autoware.ai.repos "https://raw.githubusercontent.com/Autoware-AI/autoware.ai/1.14.0/autoware.ai.repos?inline=false"
```

Download Autoware.AI repositories into the workspace, install dependencies using rosdep and compile it
```
vcs import src < autoware.ai.repos
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```


# Install workspace for AA274 lecture
Clone repository
```
git clone https://github.com/virtual-vehicle/aa274_autoware_ws.git ~/ros/aa274_autoware_ws
cd ~/ros/aa274_autoware_ws
source ~/ros/autoware.ai/devel/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
Setup for environment which includes several aliases, environment variables, and sources 
```
./setup_environment.sh
```	
**Build the workspace**

Close and open a new terminal to source the Autoware.AI components!

```
cd ~/ros/aa274_autoware_ws
catkin build
```
Extract recorded data from the racetrack
```
cd ~/ros/aa274_autoware_ws/src/arg_localization/arg_data_croix_en_ternois/bagfile
tar -xvf devbot_lap0.tar.xz
```



# Demo Localization
### Terminal 1:
```
roscore 
```

### Terminal 2:
GPS-based localization
```
roslaunch arg_launch Devbot_localization.launch
```
or Lidar-based localization
```
roslaunch arg_launch Devbot_localization.launch lidar_localization:=true
```

### Terminal 3:
```
roslaunch arg_launch Play_rosbag.launch
```
You must hit the **space key** to start to play the rosbag.

### Possible modifications can be performed:
- [Adding noise to the GPS data](https://github.com/virtual-vehicle/aa274_autoware_ws/blob/master/src/arg_localization/arg_launch/launch/Devbot_localization.launch#L14)
- [Activate/deactivate the input pose for the EKF](https://github.com/virtual-vehicle/aa274_autoware_ws/blob/master/src/arg_localization/arg_launch/launch/Devbot_localization.launch#L65)
- [Activate/deactivate the input twist for the EKF](https://github.com/virtual-vehicle/aa274_autoware_ws/blob/master/src/arg_localization/arg_launch/launch/Devbot_localization.launch#L72)
- [Modify the standard deviation parameters for the input pose](https://github.com/virtual-vehicle/aa274_autoware_ws/blob/master/src/arg_localization/arg_launch/launch/Devbot_localization.launch#L76)


# Demo Path Planning 
### Terminal 1:
```
roslaunch vifware_launch simulation.launch
```

