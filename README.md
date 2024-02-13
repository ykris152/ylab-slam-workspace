# ylab-slam-workspace
This repo is the base files for ylab mobile robot
Which includes packages for Moving the robot, Teleoperation, Gmapping, Navigation

# Install ROS-Melodic
http://wiki.ros.org/melodic/Installation/Ubuntu

# Setting up this workspace
1. Verify that you have Ubuntu 18.0 with ROS Melodic installed.
2. <code>git clone https://github.com/ykris152/ylab-slam-workspace.git</code>
3. cd ~/ylab-slam-workspace
4. catkin_make
5. echo "source ~/ylab-slam-workspace/devel/setup.bash" >> ~/.bashrc

# Connecting the robot
Connect with the correct order:
1. Robot
2. URG Sensor / RPLIDAR
3. <code>sudo chmod 666 /dev/tty*</code>

# Robot with teleoperation
## Launching Robot driver
<code>roslaunch semi_navigation driver.launch</code>

## Teleoperation node
<code>rosrun teleop_twist_keyboard teleop_twist_keyboard.py</code>

# Gmapping
1. <code>roslaunch semi_navigation all_gmapping.launch</code>
2. Launch the teleoperation node
3. Drive the robot around, and check the map generated on rviz
4. Run the following code to save the map
<code>rosrun map_server map_saver -f {name of the map}</code>

# Navigation
## Make sure you are using the correct map file.
1. Copy the <code>{name of the map}.pgm</code> and <code>{name of the map}.yaml</code> to <code>ylab-slam-workspace/src/semi_navigation/maps</code>
2. Edit <code>ylab-slam-workspace/src/semi_navigation/launch/map_server.launch</code>
3. change <code>args="$(find semi_navigation)/maps/.yaml"</code> to <code>args="$(find semi_navigation)/maps/{name of the map}.yaml"</code>

## Running Navigation
1. <code>roslaunch semi_navigation all_navigation.launch</code>
2. Open rviz window, Click 2D pose estimate to give the robot's initial location
3. Click 2D Nav Goal to send a target position.




