# Quadrotor-tracking
Centroid tracking using faster rcnn based human detector.
## Installation
1. Install Gazebo, jMAVsim simulators and Pixhawk tools, [reference](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html):
```bash
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/ubuntu.sh
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/requirements.txt

bash ubuntu.sh
```
Reboot when installation is finished:
```bash
sudo reboot
```

2. Install [ROS](http://wiki.ros.org/ROS/Installation) and [Mavros](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html):
```bash
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```
3. Clone and build PX4 [Firmware](https://dev.px4.io/v1.9.0/en/setup/building_px4.html):
```bash
mkdir ~/src; cd ~/src
git clone https://github.com/PX4/Firmware.git --recursive
cd ~/src/Firmware
make px4_sitl_default jmavsim
```
4. Create ROS catkin workspace and build the package:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Damodar-D/Quadrotor-tracking.git
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Simulation
In order to fly in a simulator:

1. Launch a simulated environment (Gazebo):
```bash
cd Firmware
make px4_sitl_default gazebo_solo
```

2. Connect to the simulated drone:
```bash
roslaunch px4_control px4_sim.launch
```

3. Execute flight command:
```bash
rosrun px4_control drone_test.py
```
## Simulation Result
![Result](https://github.com/griimmm/Quadrotor-tracking/blob/master/ezgif.com-video-to-gif.gif)
