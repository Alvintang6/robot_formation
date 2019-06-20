
#include "udp_client.h"

void Udp_com::send_udp(int fd,struct sockaddr_in addr_to, struct ip_list ip )
{
 char buf[4];
 int len;
 int count=0;



memcpy(&buf,&this->head_cback,sizeof(buf));
 
 addr_to.sin_addr.s_addr=inet_addr(ip.robot1);
 len=sendto(fd,buf,sizeof(buf),0,(struct sockaddr*)&addr_to,sizeof(addr_to)); 
 if(len==-1)
 {
 printf("send falure! for 1\n");
 }
 addr_to.sin_addr.s_addr=inet_addr(ip.robot2);
 len=sendto(fd,buf,sizeof(buf),0,(struct sockaddr*)&addr_to,sizeof(addr_to));
 if(len==-1)
 {
 printf("send falure! for 2\n");
 }

 addr_to.sin_addr.s_addr=inet_addr(ip.robot3);
 len=sendto(fd,buf,sizeof(buf),0,(struct sockaddr*)&addr_to,sizeof(addr_to));
 if(len==-1)
 {
 printf("send falure! for 3\n");
 }

 addr_to.sin_addr.s_addr=inet_addr(ip.robot4);
 len=sendto(fd,buf,sizeof(buf),0,(struct sockaddr*)&addr_to,sizeof(addr_to));
 if(len==-1)
 {
 printf("send falure! for 4\n");
 }

// not use in this project(infomation back to laptop)
 //addr_to.sin_addr.s_addr=inet_addr(ip.laptop);
 //len=sendto(fd,buf,sizeof(buf),0,(struct sockaddr*)&addr_to,sizeof(addr_to));
 //if(len==-1)
 //{
 //printf("send falure! for laptop\n");
 //} 

 //else
 //{
 //printf("%d bytes sended successfully! and it is %f.\n",len, head_cback);

 //}


}



void Udp_com::imu_CB(const sensor_msgs::Imu::ConstPtr& msg){
	///	
	
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);
	head_cback=yaw;
//printf("head calculated %f \n",head_cback);
	
//std::cout<< "roll =" << roll << "pitch = " << pitch << "yaw" << std::endl;

}

