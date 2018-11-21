#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "control_bot.h"
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv) {


       float desired_v0,desired_h,motor_lim;  //where desired_h is offset value and should in radain uint
       float kvij,kcij;

	ros::init(argc, argv, "cam_pub");
	Ctrl_bot obj; 
        ros::param::get("~/desired_v",desired_v0);
	ros::param::get("~/desired_h",desired_h);
	ros::param::get("~/motor_lim",motor_lim);
	ros::param::get("~/k_vij",kvij);
	ros::param::get("~/k_cij",kcij);

	obj.designed={0.8,0.3,1.6,0};	// robot1,robot3 position
 	ros::Rate rate(16);


 while(ros::ok()){


//if(obj.pc_ctrl==1){
    geometry_msgs::Twist msg;    
    obj.total= obj.total_gradient(obj.designed,kvij,kcij);
    obj.cmd = obj.vel_calculate(obj.total,motor_lim,desired_v0 ,desired_h); // (0,0)

    msg.linear.x = obj.cmd.linear; msg.angular.z=obj.cmd.angular;
   printf("obj.cmd.linear= %f \n \n",obj.cmd.linear);

    obj.pub.publish(msg);   
 // }
rate.sleep();
ros::spinOnce();

}



return 0;
}
