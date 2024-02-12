#!/bin/bash

# Update the package list
sudo apt update

# Upgrade all installed packages
sudo apt upgrade -y

# Perform a distribution upgrade
sudo apt dist-upgrade -y

# Remove unnecessary packages and clean up
sudo apt --purge autoremove -y
sudo apt autoclean -y

#ROS stuffs
#Set up keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#Install Desktop-Full Install
sudo apt install -y ros-melodic-desktop-full 

#Environment Setup
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#Dependencies for building packages
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

#Initialize rosdep
sudo rosdep init
rosdep update