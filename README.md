# bumble
Main controller node for the robot we built, as a ROS Melodic project.

# Installing ROS

First, follow the steps at http://wiki.ros.org/melodic/Installation/Ubuntu, and get through the following steps:
 1. Installation use `ros-melodic-ros-base`
 2. Environment Setup
 3. Dependencies for building packages

Next, you'll need to set up a catkin workspace:
```shell
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/
catkin_make
cd ~/catkin_ws/src
git clone https://github.com/GoodDogAI/bumble.git

# Install libbluetooth
sudo apt-get install libbluetooth-dev
```
