#include <ros/ros.h>
#include "myturtle/pathplanner.h"

bool pathplan(myturtle::pathplanner::Request &request, myturtle::pathplanner::Response &response)
{
  geometry_msgs::Pose poseTmp;


  // 1. goal point
  poseTmp.position.x = 4;
  poseTmp.position.y = 6;
  poseTmp.position.z = 0;
  response.path.poses.push_back(poseTmp);
  //2. goal point
  poseTmp.position.x = 10;
  poseTmp.position.y = 6;
  poseTmp.position.z = 0;
  response.path.poses.push_back(poseTmp);
  //3. goal point
  poseTmp.position.x = 10;
  poseTmp.position.y = 2;
  poseTmp.position.z = 0;
  response.path.poses.push_back(poseTmp);


  ROS_INFO(" sending back response");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_planner_server");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("path_planner", pathplan);
  ros::spin();
  return 0;
}
