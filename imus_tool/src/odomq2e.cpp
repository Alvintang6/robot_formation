

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>





void quater_CB(const nav_msgs::Odometry::ConstPtr& msg){
	///	
	float head_cback;
	
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);
	head_cback=yaw;
printf("angular_value calculated %f \n",head_cback);
	
//std::cout<< "roll =" << roll << "pitch = " << pitch << "yaw" << std::endl;

}



int main(int argc, char **argv){

 std::string topic_n="/odom";

  ros::param::get("~/topic_name",topic_n);
  
  char topic[30];

 if(topic[3]== NULL)
{
printf("please enter the topic of the quatertion ");
}

strcpy(topic,topic_n.c_str());

 ros::init(argc, argv, "quater2ang");
 ros::NodeHandle n;

 ros::Subscriber sub = n.subscribe(topic, 1000, quater_CB);

 ros::spin();





return 0;
 }










