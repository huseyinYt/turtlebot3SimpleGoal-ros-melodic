#include "myturtle/victoryBehaviorAction.h"
#include <actionlib/server/simple_action_server.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <cmath>

typedef actionlib::SimpleActionServer<myturtle::victoryBehaviorAction> Server;
nav_msgs::Odometry turtlebotPosition;
ros::Subscriber odomSubscriber;
ros::Publisher velPublisher;
void odomCallBack(const nav_msgs::Odometry &msg)
{
    turtlebotPosition = msg;
}
void turn(double speed)
{
    geometry_msgs::Twist message;
    message.linear.x = 0;
    message.linear.y = 0;
    message.linear.z = 0;
    message.angular.x = 0;
    message.angular.y = 0;
    message.angular.z = speed;
    velPublisher.publish(message);
}

void execute(const myturtle::victoryBehaviorGoalConstPtr &goal, Server *as)
{
    const double startYaw = goal.get()->yaw; // startYaw and turn 2pi radian
    double currentYaw = 0;
    double difValue = 1; ;
    bool success = false;
    myturtle::victoryBehaviorFeedback feedback;
    tf::Pose pose;
    ros::Rate r(1);

    turn(1.0);
    ros::spinOnce();

    // publish info to the console for the user
    ROS_INFO("victory behavior: Executing...");
    r.sleep();

    while (abs(difValue) > 0.05)
    {
        turn(1.0);
        ros::spinOnce();
        tf::poseMsgToTF(turtlebotPosition.pose.pose, pose);
        currentYaw = tf::getYaw(pose.getRotation());
        feedback.currentYaw = currentYaw;
        as->publishFeedback(feedback);

        difValue = currentYaw - startYaw;
    }
    turn(0.0);
    ros::spinOnce();
    success = true;
    // check that preempt has not been requested by the client
    if (as->isPreemptRequested() || !ros::ok())
    {

        ROS_INFO("victory behavior: Preempted");
        // set the action state to preempted
        as->setPreempted();
        success = false;
    }
    r.sleep();
    myturtle::victoryBehaviorResult result;
    if (success)
    {
        result.ok = true;
        ROS_INFO("victory behavior: Succeded");
        // set the action state to succeeded
        as->setSucceeded(result);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "victory_behavior_server");
    ros::NodeHandle n;
    odomSubscriber = n.subscribe("odom", 1000, odomCallBack);
    velPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    Server server(n, "victory", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}
