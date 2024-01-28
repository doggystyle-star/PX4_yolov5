# PX4_yolov5
基于yolov5的无人机仿真平台px4的目标跟踪控制

该仓库基于
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) and [xtdrone](https://github.com/robin-shaun/XTDrone)仿真平台,安装px4请参照[xtdrone仿真平台配置](https://www.yuque.com/xtdrone/manual_cn/basic_config_13)
请确保满足[yolov5安装条件](https://github.com/ultralytics/yolov5)
<div align="center">
    <img src="pic/output.gif" width="1024" height="640"/>
</div>

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 18.04** (18.04版本请将noetic替换为melodic)

### 1.1 **安装ROS1**
  * `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
  * `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
  * `sudo apt update`
  * `sudo apt install ros-noetic-desktop-full`
  * `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`

  ### 1.2 **安装gazebo**
  * `sudo apt-get install ros-noetic-moveit-msgs ros-noetic-object-recognition-msgs ros-noetic-octomap-msgs ros-noetic-camera-info-manager  ros-noetic-control-toolbox ros-noetic-polled-camera ros-noetic-controller-manager ros-noetic-transmission-interface ros-noetic-joint-limits-interface`
  * `mkdir -p ~/catkin_ws/src`
  * `cd ~/catkin_ws/src`
  * `git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git`
  * `cd ~/catkin_ws`
  * `catkin_make`

### 1.3 **安装realsense**
  * `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`
  * `sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u`
  * `sudo apt-get install librealsense2-dkms`
  * `sudo apt-get install librealsense2-utils`
  * `sudo apt-get install librealsense2-dev`
  * `sudo apt-get install librealsense2-dbg`
  * 测试 `realsense-viewer` 或 `roslaunch realsense2_camera rs_camera.launch`
### 1.4 **安装Mavros**
  * `sudo apt-get install ros-noetic-mavros`
  * `cd /opt/ros/noetic/lib/mavros`
  * `sudo ./install_geographiclib_datasets.sh`

### 1.5 **安装PX4**
  * `git clone https://github.com/PX4/PX4-Autopilot.git`
  * `mv PX4-Autopilot PX4_Firmware`
  * `cd PX4_Firmware`
  * `git checkout -b xtdrone/dev v1.13.2`
  * `git submodule update --init --recursive`
  * `make px4_sitl_default gazebo`

*注意:*
- 安装px4请参照[xtdrone仿真平台配置](https://www.yuque.com/xtdrone/manual_cn/basic_config_13)
- 修改 ~/.bashrc，加入以下代码,注意路径匹配。
- `source ~/catkin_ws/devel/setup.bash`
- `source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default`
- `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware`
- `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo`
- `export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/catkin_ws/`

## 2.Build from source
### 2.1 **克隆仓库**
  * `cd ~/catkin_ws/src`
  * `git clone https://github.com/doggystyle-star/PX4_yolov5.git`
  * `mv PX4_yolov5/models ~/.gazebo`
  * `cd PX4_yolov5`

### 2.2 **更改启动文件**
* 修改启动脚本文件
* `cp sitl_config/init.d-posix/* ~/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/`
* 添加launch文件
* `cp -r sitl_config/launch/* ~/PX4_Firmware/launch/`
* 添加世界文件
* `cp sitl_config/worlds/* ~/PX4_Firmware/Tools/sitl_gazebo/worlds/`
* 修改部分插件
* `cp sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.cpp ~/PX4_Firmware/Tools/sitl_gazebo/src`
* `cp sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.hh ~/PX4_Firmware/Tools/sitl_gazebo/include`
* `cp sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.cpp ~/PX4_Firmware/Tools/sitl_gazebo/src`
* `cp sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.h ~/PX4_Firmware/Tools/sitl_gazebo/include`
* 修改CMakeLists.txt
* `cp sitl_config/CMakeLists.txt ~/PX4_Firmware/Tools/sitl_gazebo`
* 修改部分模型文件
* `cp -r sitl_config/models/* ~/PX4_Firmware/Tools/sitl_gazebo/models/ `
### 2.3 **删除原编译文件，重新编译PX4固件**
* `cd ~/PX4_Firmware`
* `rm -r build/ devel/`
* `make px4_sitl_default gazebo`

### 2.4 **编译yolov5_ros**

* `cd ~/catkin_ws/src`
* `mv PX4_yolov5/yolov5_ros /yolov5_ros`
* `cd ..`
* `catkin_make`


## 3.Deirctly Run
*注意:*

如果没有将工作空间加入.bashrc在使用之前请记得 `source devel/setup.bash`
### Start With the Command.
**Start  px4**
  
**首先加载世界环境:**

```
  roslaunch px4 outdoor1.launch
```

**其次运行yolov5_ros:**

```
  roslaunch yolov5_ros yolov5.launch
```
**建立通信:**
```
  cd ~/catkin_ws/src/PX4_yolov5/communication/
  python3 multirotor_communication.py iris 0
```
**使用键盘控制飞机按照提示将飞机飞到目标附近**
```
  cd ~/catkin_ws/src/PX4_yolov5/control/keyboard/
  python3 multirotor_keyboard_control.py iris 1 vel
```
**关闭键盘控制，启动跟踪控制**
```
  cd ~/catkin_ws/src/PX4_yolov5/control/
  python3 tracking_PID_500.py iris 0
```

## Reference
* YOLOv5 official repository: https://github.com/ultralytics/yolov5
* YOLOv5_ROS repository: https://github.com/mats-robotics/yolov5_ros
