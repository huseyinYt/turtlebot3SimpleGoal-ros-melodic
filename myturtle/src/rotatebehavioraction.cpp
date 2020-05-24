#include "myturtle/rotateBehaviorAction.h"
#include <actionlib/server/simple_action_server.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <cmath>
typedef actionlib::SimpleActionServer<myturtle::rotateBehaviorAction> Server;
nav_msgs::Odometry turtlebotPosition;
ros::Subscriber odomSubscriber;
ros::Publisher velPublisher;
void odomCallBack(const nav_msgs::Odometry &msg)
{
  turtlebotPosition = msg;
}

void turnControl(double speed)
{
  geometry_msgs::Twist message;
  message.linear.x = 0.0;
  message.linear.y = 0.0;
  message.linear.z = 0.0;
  message.angular.x = 0.0;
  message.angular.y = 0.0;
  message.angular.z = speed;
  velPublisher.publish(message);
}
void execute(const myturtle::rotateBehaviorGoalConstPtr &goal, Server *as) // Note: "Action" is not appended to DoDishes here
{
  const double angleToturn = goal.get()->angle;
  double yawValue;
  double rotatedAngle;
  // helper variables
  ros::Rate r(1);
  tf::Pose pose;
  bool success = false;
  myturtle::rotateBehaviorFeedback feedback;
  
  // publish info to the console for the user
  ROS_INFO("rotate behavior: Executing...");

  // start executing the action
  tf::poseMsgToTF(turtlebotPosition.pose.pose, pose);
  const double startAngle = tf::getYaw(pose.getRotation());
  double difValue = angleToturn;

  // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
  r.sleep();

  ROS_INFO("startAngle: %f", startAngle);

  if (angleToturn < 0)
  {
    while (difValue < 0.000)
    {
      turnControl(-0.1); //-- turn right
      ros::spinOnce();

      // publish the feedback
      tf::poseMsgToTF(turtlebotPosition.pose.pose, pose);
      yawValue = tf::getYaw(pose.getRotation());
      rotatedAngle = yawValue - startAngle;
      difValue = angleToturn - rotatedAngle;

      feedback.remainingAngle = difValue; 
      as->publishFeedback(feedback);

      ROS_INFO("yawValue %f rotatedAngle %f angleToturn %f difValue %f ", yawValue, rotatedAngle, angleToturn, difValue);
    }
  }
  else
  {
    while (difValue > 0.000)
    {
      turnControl(0.1); //-- turn left
      ros::spinOnce();

      // publish the feedback
      tf::poseMsgToTF(turtlebotPosition.pose.pose, pose);
      yawValue = tf::getYaw(pose.getRotation());
      rotatedAngle = yawValue - startAngle;
      difValue = angleToturn - rotatedAngle;

      feedback.remainingAngle = difValue; //burada kalan dönme atanacak
      as->publishFeedback(feedback);

      ROS_INFO("yawValue %f rotatedAngle %f angleToturn %f difValue %f ", yawValue, rotatedAngle, angleToturn, difValue);
    }
  }

  turnControl(0.0);
  ros::spinOnce();
  success = true;

  // check that preempt has not been requested by the client
  if (as->isPreemptRequested() || !ros::ok())
  {

    ROS_INFO("move behavior: Preempted");
    // set the action state to preempted
    as->setPreempted();
    success = false;
  }

  myturtle::rotateBehaviorResult result;
  if (success)
  {
    result.ok = true;
    ROS_INFO("rotate behavior: Succeeded");
    // set the action state to succeeded
    as->setSucceeded(result);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotate_behavior_server");
  ros::NodeHandle n;
  odomSubscriber = n.subscribe("odom", 1000, odomCallBack);
  velPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  Server server(n, "rotate", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
