
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>

static bool first_frame=0;
static long int first_header;

void quater_CB(const sensor_msgs::Imu::ConstPtr& msg){
	///	
	float head_cback;

if(first_frame==0){
first_frame=1;
first_header= msg->header.seq;
}

	tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);

	head_cback = yaw*57.295779;
printf("angular_value calculated %f \n",head_cback);
	
//std::cout<< "roll =" << roll << "pitch = " << pitch << "yaw" << std::endl;

}




int main(int argc, char **argv){

 std::string topic_n="/imu/data_raw";

  ros::param::get("~/topic_name",topic_n);
  
  char topic[30];

 if(topic[3]== NULL)
{
printf("you can enter yourown topic of the quatertion ");
}

strcpy(topic,topic_n.c_str());

 ros::init(argc, argv, "quater2ang");
 ros::NodeHandle n;

 ros::Subscriber sub = n.subscribe(topic, 1000, quater_CB);

 ros::spin();





return 0;
 }

