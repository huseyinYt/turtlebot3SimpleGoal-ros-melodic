#include "myturtle/moveBehaviorAction.h"
#include <actionlib/server/simple_action_server.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <cmath>
typedef actionlib::SimpleActionServer<myturtle::moveBehaviorAction> Server;
nav_msgs::Odometry turtlebotPosition;
ros::Subscriber odomSubscriber;
ros::Publisher velPublisher;
void odomCallBack(const nav_msgs::Odometry &msg)
{
  turtlebotPosition = msg;
}

void move(double speed)   
{
  geometry_msgs::Twist message;
  message.linear.x = speed;
  message.linear.y = 0.0;
  message.linear.z = 0.0;
  message.angular.x = 0.0;
  message.angular.y = 0.0;
  message.angular.z = 0.0;
  velPublisher.publish(message);
}

void execute(const myturtle::moveBehaviorGoalConstPtr &goal, Server *as) // Note: "Action" is not appended to DoDishes here
{
  const double distanceToGo = goal.get()->distance;
  double distanceValue = 0;
  double difValue;
  const double xGoal = turtlebotPosition.pose.pose.position.x;
  const double yGoal = turtlebotPosition.pose.pose.position.y;
  bool success = false;

  myturtle::moveBehaviorFeedback feedback;
  tf::Pose pose;

  // helper variables
  ros::Rate r(1);

  // publish info to the console for the user
  ROS_INFO("move behavior: Executing...");
  distanceValue = sqrt(pow((xGoal - turtlebotPosition.pose.pose.position.x), 2) + pow((yGoal - turtlebotPosition.pose.pose.position.y), 2));
  difValue = distanceToGo - distanceValue;

  while (difValue > 0.0)
  {
    move(0.1);
    ros::spinOnce();

    distanceValue = sqrt(pow((xGoal - turtlebotPosition.pose.pose.position.x), 2) + pow((yGoal - turtlebotPosition.pose.pose.position.y), 2));
    difValue = distanceToGo - distanceValue;
    feedback.remainingDistance = difValue; 
    // publish the feedback
    as->publishFeedback(feedback);
    ROS_INFO("distanceValue = %f difValue : %f distanceToGo %f  ", distanceValue, difValue, distanceToGo);
  }

  move(0.0);
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

  feedback.remainingDistance = 0; //burada kalan uzaklık atanaca
  // publish the feedback
  as->publishFeedback(feedback);
  // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
  r.sleep();

  myturtle::moveBehaviorResult result;
  if (success)
  {
    result.ok = true;
    ROS_INFO("move behavior: Succeeded");
    // set the action state to succeeded
    as->setSucceeded(result);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_behavior_server");
  ros::NodeHandle n;
  odomSubscriber = n.subscribe("odom", 1000, odomCallBack);
  velPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  Server server(n, "move", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
