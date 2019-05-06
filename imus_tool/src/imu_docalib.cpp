#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>

static long int count_i=0;
static double heading_first;
static long int num_first;
static float bias_total;



void quater_CB(const sensor_msgs::Imu::ConstPtr& msg){
	///	
	double bias_frame;
	long int seq_num;
	tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);
	
if(count_i==0)
{
heading_first=yaw;
num_first = msg->header.seq;
}
	
	count_i++;
	
seq_num=msg->header.seq;


if(msg->header.seq%500==0)
{
printf("data processing....");
}

if(msg->header.seq-(num_first+10000)>=0){

bias_total=yaw-heading_first;
bias_frame=bias_total/10000.0;
printf("total_bias %f, bias to compensate %f \n",bias_total, bias_frame);
exit(0);
}
	
//std::cout<< "roll =" << roll << "pitch = " << pitch << "yaw" << std::endl;

}





int main(int argc, char **argv){

 //std::string topic_n="/imu/data_raw";

  //ros::param::get("~/topic_name",topic_n);
  
  //char topic[30];

 //if(topic[3]== NULL)
//{
//printf("you can enter yourown topic of the quatertion ");
//}

//strcpy(topic,topic_n.c_str());

 ros::init(argc, argv, "quater2ang");
 ros::NodeHandle n;

 ros::Subscriber sub = n.subscribe("/imu/data_raw", 1000, quater_CB);

 ros::spin();





return 0;
 }

