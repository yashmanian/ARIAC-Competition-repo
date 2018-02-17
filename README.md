# ENPM809B
Assignment files for ENPM809B

# Assignment 1
Make the kobuki base move around randomly in the Gazebo world
- Run "roslaunch turtlebot_gazebo turtlebot_world.launch"
- cd to ROS workspace and source in a separate terminal.
- Run "rosrun turtlebot_demo random_vel"

# Assignment 2 to do
- Configure yaml files to populate as required
- Figure out pick and place from the bins
- Figure out the dropped parts test
Pick and place breakdown
- Query sensor on the bin, to get the number of parts and their pose on the bin
- Query the order topic to find out what parts you need to pick and place.
- Plan out order of pick and place.
- Execute to drop parts onto the AGV.
