//////////////////////////////////////////////////////////////////////
//
//  This is a udp_server node to get the information from other nodes.
//  It also publish the received information to local ros master with topic 
//  (/cross_info). 
//      author: JieTang                 data: 29/08/2018
//
//////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <iostream>
#include "path_calculate/heading.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <netinet/in.h>//for sockaddr_in
#include <arpa/inet.h>

struct info_recv{
	float robot1;
	float robot2;
	float robot3;
	int laptop;
};

struct info_recv udp_info;

struct arg_pass{	
	char robot1[14];
 	char robot2[14];
	char robot3[14];
	char laptop[14];
	int fd; 

};


void* recv_thread(void *arg){
	int r;
	
	struct arg_pass arg_inside;
	int upd_fd;
	char buf[4];	

	arg_inside = *(struct arg_pass *)arg;

	 struct sockaddr_in from;  
 		socklen_t len;
 		len=sizeof(from);
	
  while(1)
  {
	
 	r=recvfrom(arg_inside.fd,buf,sizeof(buf),0,(struct sockaddr*)&from,&len);
printf("in while r=%d, addr%s \n",r,inet_ntoa(from.sin_addr));
	
     if(r>0)
     { 
	char* ip = inet_ntoa(from.sin_addr); 
	
	if(strcmp(ip,arg_inside.robot1)==0)
	{
       	  memcpy(&udp_info.robot1,buf,sizeof(float));
	}
	else if(strcmp(ip,arg_inside.robot2)==0)
	{
	  memcpy(&udp_info.robot2,buf,sizeof(float));
	}
	else if(strcmp(ip,arg_inside.robot3)==0)
	{
	  memcpy(&udp_info.robot3,buf,sizeof(float));
	}
	else if(strcmp(ip,arg_inside.laptop)==0)
	{
	  memcpy(&udp_info.laptop,buf,sizeof(int));
	}
	else{continue;}

 	printf("The message received for %s is :ip1:%f,ip2:%f,ip3:%f,laptop:%d \n",inet_ntoa(from.sin_addr),udp_info.robot1,udp_info.robot2,udp_info.robot3,udp_info.laptop);
     } 
     else
     {
	printf("r<0 \n");
     break ;
     }
 
}
close(arg_inside.fd);
 return 0;
}






int main(int argc, char **argv) {

ros::init(argc, argv, "multi_udpsrv"); 
ros::NodeHandle nh;

ros::Publisher pub = nh.advertise<path_calculate::heading>("/cross_info",1000);



 int udp_fd=socket(AF_INET,SOCK_DGRAM,0);
 if(udp_fd==-1)
 {
 perror("socket create error!\n");
 exit(-1);
 }
 ROS_INFO("socket fd=%d\n",udp_fd);
 
//const int opt =1;
//int nb=0;
//nb=setsockopt(udp_fd,SOL_SOCKET,SO_BROADCAST,(char*)&opt,sizeof(opt)); 

//if(nb==-1)
//{
 //perror("socket create error!\n");
 //exit(-1);
 //}
 std::string local_ip1;
 char local_ip[14];
 ros::param::get("/local_ip",local_ip1);
 strcpy(local_ip,local_ip1.c_str());

 struct sockaddr_in addr;
 addr.sin_family=AF_INET;
 addr.sin_port=htons(10278);
 addr.sin_addr.s_addr=inet_addr(local_ip); // always bind loacl ip address
 
 
 int r;
 r=bind(udp_fd,(struct sockaddr*)&addr,sizeof(addr));
 if(r==-1)
 {
 ROS_INFO("Bind error!\n");
 close(udp_fd);
 exit(-1);
 }
 ROS_INFO
("Bind successfully.\n");
 

std::string bot1_ip,bot2_ip,bot3_ip,laptop_ip;

ros::param::get("/robot1_ip",bot1_ip);
ros::param::get("/robot2_ip",bot2_ip);
ros::param::get("/robot3_ip",bot3_ip);
ros::param::get("/laptop_ip",laptop_ip);

	

arg_pass arg_udpthread;  // SET IP for different robot

strcpy(arg_udpthread.robot1,bot1_ip.c_str());
strcpy(arg_udpthread.robot2,bot2_ip.c_str());
strcpy(arg_udpthread.robot3,bot3_ip.c_str());
strcpy(arg_udpthread.laptop,laptop_ip.c_str());
arg_udpthread.fd=udp_fd;

printf("robot3_ip=%s \n", arg_udpthread.robot3);


	
 
	//obj.robotlist={-0.25,0.25,-2,0};	

 	ros::Rate rate(20);
        
	pthread_t udp_recv;
	int err;

 if((err = pthread_create(&udp_recv,NULL, recv_thread,(void*)&arg_udpthread))!=0){
	perror("pthread_create error");
}


 while(ros::ok()){

    path_calculate::heading msg;
    
	//if(udp_info.robot3 != NULL )
    {msg.robot1 = udp_info.robot1; msg.robot2= udp_info.robot2; 
    msg.robot3 = udp_info.robot3; msg.laptop= udp_info.laptop;
   

    pub.publish(msg);
    rate.sleep();
   }
 }

return 0;
}
