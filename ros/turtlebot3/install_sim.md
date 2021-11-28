# TurtleBot 3

# Installing TurtleBot3 Simulation Software

TurtleBot3 QuickStart Guide: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/ 

Here, we assume that your computer is running Ubuntu 18.04 and that ROS-melodic has already been installed. If not, the link to the quick start guide contains instructions and an installation script for this process.

## Install Dependent ROS packages

```bash
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
ros-melodic-rosserial-server ros-melodic-rosserial-client \
ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
ros-melodic-compressed-image-transport ros-melodic-rqt* \
ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```

</br></br>

## Install Turtlebot3 Packages

```bash
sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt-get install ros-melodic-turtlebot3-*
```

</br></br>

## Set Turtlebot3 Model Name

Set the default TURTLEBOT3_MODEL name to your model.

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

Move onto the [Gazebo Simulation Worlds](gazebo_worlds.md) tutorial.
