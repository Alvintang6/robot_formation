#include "ros/ros.h"
#include <nav_msgs/Odometry.h>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);

  ros::Rate loop_rate(20);


  while (ros::ok())
  {
   
    nav_msgs::Odometry msg;

    msg.pose.pose.orientation.x=0 ;
 msg.pose.pose.orientation.y=0 ;
 msg.pose.pose.orientation.z=0 ;
 msg.pose.pose.orientation.w=0 ;

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
