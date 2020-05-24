#include "ros/ros.h"
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include "myturtle/pathplanner.h"
#include "myturtle/moveBehaviorAction.h"
#include "myturtle/rotateBehaviorAction.h"
#include "myturtle/victoryBehaviorAction.h"
#include <cstdlib>
#include "nav_msgs/Odometry.h"
#include <cmath>
nav_msgs::Odometry turtlebotPosition;
enum STATE
{
  STATE_PLAN = 0,
  STATE_ROTATE,
  STATE_FORWARD,
  STATE_ERROR_CONTROL,
  STATE_NONE
} state;
void odomCallBack(const nav_msgs::Odometry &msg)
{

  turtlebotPosition = msg;
}

typedef actionlib::SimpleActionClient<myturtle::moveBehaviorAction> MoveBehavior;
typedef actionlib::SimpleActionClient<myturtle::rotateBehaviorAction> RotateBehavior;
typedef actionlib::SimpleActionClient<myturtle::victoryBehaviorAction> VictoryBehavior;

int main(int argc, char **argv)
{

  geometry_msgs::PoseArray path;

  ros::init(argc, argv, "myturtlecontrol");
  ros::NodeHandle n;
  ros::Subscriber odomSubscriber;
  const double errorTolerance = 0.03; // %3 error tolerance
  double xGoalPoint, yGoalPoint, yawGoal, yawValue, x, y, distance;

  odomSubscriber = n.subscribe("odom", 1000, &odomCallBack);
  MoveBehavior moveBehavior("move", true);
  moveBehavior.waitForServer();
  ROS_INFO("move_behavior is connected...");
  RotateBehavior rotateBehavior("rotate", true);
  rotateBehavior.waitForServer();
  ROS_INFO("rotate_behavior is connected...");
  VictoryBehavior victoryBehavior("victory", true);
  victoryBehavior.waitForServer();
  ROS_INFO("victory_behavior is connected...");

  myturtle::moveBehaviorGoal moveGoal;
  myturtle::rotateBehaviorGoal rotateGoal;
  myturtle::victoryBehaviorGoal victoryGoal;
  tf::Pose pose;

  //Once hedef icin plani al.
  ros::ServiceClient client = n.serviceClient<myturtle::pathplanner>("path_planner");
  myturtle::pathplanner srv;
  srv.request.pose.position.x = 0; // hedef konum olacak
  srv.request.pose.position.y = 0; // hedef konum olacak
  srv.request.pose.position.z = 0; // hedef konum olacak

  if (client.call(srv))
  {

    path = srv.response.path;
    ROS_INFO("waypoints are received...");
  }
  else
  {
    ROS_ERROR("Failed to call service path plan");
    return 1;
  }

  for (auto &goal : path.poses)
  {
    state = STATE_PLAN;
    while (state != STATE_NONE && ros::ok())
    {
      ros::spinOnce();
      switch (state)
      {
      case STATE_PLAN:
        ROS_INFO("STATE_PLAN");
        xGoalPoint = goal.position.x - turtlebotPosition.pose.pose.position.x;
        yGoalPoint = goal.position.y - turtlebotPosition.pose.pose.position.y;
        ROS_INFO("goal.x: %f goal.y: %f turtlebotPosition.x: %f turtlebotPosition.y: %f", goal.position.x, goal.position.y, turtlebotPosition.pose.pose.position.x, turtlebotPosition.pose.pose.position.y);
        yawGoal = atan2(yGoalPoint, xGoalPoint);
        tf::poseMsgToTF(turtlebotPosition.pose.pose, pose);
        yawValue = tf::getYaw(pose.getRotation());
        rotateGoal.angle = yawGoal - yawValue;
        ROS_INFO("yawGoal: %f yawValue: %f angleToTurn : %f ", yawGoal, yawValue, rotateGoal.angle);
        state = STATE_ROTATE;
        break;
      case STATE_ROTATE:
        ROS_INFO("STATE_ROTATE");
        rotateBehavior.sendGoal(rotateGoal);
        rotateBehavior.waitForResult();
        if (rotateBehavior.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("rotate behavior succeeded..");
          state = STATE_FORWARD;
        }
        else
        {
          ROS_INFO("rotate behavior unsucceeded..");
          state = STATE_NONE;
        }
        break;
      case STATE_FORWARD:
        ROS_INFO("STATE_FORWARD");
        moveGoal.distance = sqrt(pow(xGoalPoint, 2) + pow(yGoalPoint, 2));
        ROS_INFO("distance %f", moveGoal.distance);
        moveBehavior.sendGoal(moveGoal);
        moveBehavior.waitForResult();
        if (moveBehavior.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("move behavior succeeded..");
          state = STATE_ERROR_CONTROL;
        }
        else
        {
          ROS_INFO("move behavior unsucceeded..");
          state = STATE_NONE;
        }
        break;
      case STATE_ERROR_CONTROL:
        ROS_INFO("STATE_ERROR_CONTROL");
        x = goal.position.x - turtlebotPosition.pose.pose.position.x;
        y = goal.position.y - turtlebotPosition.pose.pose.position.y;
        distance = sqrt(pow(x, 2) + pow(y, 2));
        tf::poseMsgToTF(turtlebotPosition.pose.pose, pose);
        distance = tf::getYaw(pose.getRotation());

        if (abs(distance-yawGoal) > errorTolerance)
        {
          state = STATE_PLAN;
          ROS_INFO("error is occured..");
          ROS_INFO("error distance: %f", distance);
        }
        else
        {
          state = STATE_NONE;
        }
        break;

      default:
        break;
      }
    }
  }

  // After pass all goal point , turn around own 1 times
  ROS_INFO("******Victory*****");
  tf::poseMsgToTF(turtlebotPosition.pose.pose, pose);
  victoryGoal.yaw = tf::getYaw(pose.getRotation());
  victoryBehavior.sendGoal(victoryGoal);
  victoryBehavior.waitForResult();
  if (victoryBehavior.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("victory is succeed..");
  }
  else
  {
    ROS_INFO("victory is failed..");
  }

  ros::spin();

  return 0;
}
