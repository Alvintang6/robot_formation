////////////////////////////////////////////////////////
//
//  This is a udp_client node for subscribe the loacl topics and than 
//  send it to the internet. the ip address and the port num should be 
//  configured.
//  Author: JieTang                 date: 29/08/2018
//
////////////////////////////////////////////////////////

#include "udp_client.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_udpclt"); 
  ros::NodeHandle nh;
  Udp_com obj;


// import the ip configure from the launch file
  std::string bot1_ip,bot2_ip,bot3_ip,bot4_ip,laptop_ip;

  ros::param::get("/robot1_ip",bot1_ip);
  ros::param::get("/robot2_ip",bot2_ip);
  ros::param::get("/robot3_ip",bot3_ip);
  ros::param::get("/robot4_ip",bot4_ip);
  ros::param::get("/laptop_ip",laptop_ip);
  
  ip_list ip;    // initial the ip struct for futher use

strcpy(ip.robot1,bot1_ip.c_str());
strcpy(ip.robot2,bot2_ip.c_str());
strcpy(ip.robot3,bot3_ip.c_str());
strcpy(ip.robot4,bot4_ip.c_str());
strcpy(ip.laptop,laptop_ip.c_str());



int fd=socket(AF_INET,SOCK_DGRAM,0);
 if(fd==-1)
 {
 perror("socket create error!\n");
 exit(-1);
 }
 printf("socket fd=%d\n",fd);
 
 
 struct sockaddr_in addr_from;
 addr_from.sin_family=AF_INET;
 addr_from.sin_port=htons(0);  // get arbitrary port   
 addr_from.sin_addr.s_addr=htons(INADDR_ANY); // get the host ip

//const int opt=1;
//int nb = 0;
//nb=setsockopt(fd,SOL_SOCKET,SO_BROADCAST,(char*)&opt,sizeof(opt));
//if(nb==-1)
//{
//printf("set socket error \n");
//exit(-1);
//}


int r; 
r=bind(fd,(struct sockaddr*)&addr_from,sizeof(addr_from));

if(r<0)
 {
perror("bind error! \n");
exit(-1);
 }




 struct sockaddr_in addr_to;//
 addr_to.sin_family=AF_INET;
 addr_to.sin_port=htons(10278);
 addr_to.sin_addr.s_addr=inet_addr(ip.robot1); // unicast 
//addr_to.sin_addr.s_addr=htonl(INADDR_BROADCAST); //broadcast


ros::Rate rate(20);



while(ros::ok())
{
 obj.send_udp(fd,addr_to,ip);
rate.sleep();
ros::spinOnce();
}



}



//need function send_udp , callback for imu 
