# turtlebot3SimpleGoal-ros-melodic
Turtlebot goes some goal point by using ros actions and ros services.<br/>
Gazebo is used as a simulation tool.<br/>
Rviz is used for visualize map. (Slam - gmapping)<br/>


Start turtlebot3 in gazebo enviroment <br/>
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch <br/>

Start example <br/> 
roslaunch myturtle myturtlenodes.launch <br/>
rosrun myturtle myturtlecontrolnode <br/>

Start Rviz (if you want visualize enviroment) <br/>
roslaunch myturtle slamGmapping.launch<br/>
-- Add map topic-- <br/>

References : <br/>
https://github.com/ROBOTIS-GIT/turtlebot3<br/>
https://github.com/ros-perception/slam_gmapping<br/>
http://wiki.ros.org/actionlib<br/>
