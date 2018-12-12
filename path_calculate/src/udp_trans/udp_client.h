#ifndef _UDP_CLINET_H
#define _UDP_CLINET_H

#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <netinet/in.h>//for sockaddr_in
#include <arpa/inet.h>//for socket

struct ip_list{	
	 char robot1[14];
 	 char robot2[14];
	 char robot3[14];
	 char laptop[14];

	};


class Udp_com{


public: 

	void imu_CB(const sensor_msgs::Imu::ConstPtr& msg);
	void send_udp(int fd,struct sockaddr_in addr_to, struct ip_list ip);

	ros::Publisher headings;
	
	Udp_com(){
	self_heading = nh.subscribe("/imu/data",1000, &Udp_com::imu_CB, this );
	}

	


private:
	
	float head_cback=0;    // for store the callback function value
	
 	ros::Subscriber self_heading;    // callback for imu 
	ros::NodeHandle nh;


};




#endif
