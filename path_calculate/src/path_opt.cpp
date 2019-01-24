#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "control_bot.h"
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv) {


       float desired_v0,desired_h,motor_lim;  //where desired_h is offset value and should in radain uint
       float kvij,kcij,threshold;
	bool test=0;

	ros::init(argc, argv, "cam_pub");
	Ctrl_bot obj; 
        ros::param::get("~/desired_v",desired_v0);
	ros::param::get("~/desired_h",desired_h);
	ros::param::get("~/motor_lim",motor_lim);
	ros::param::get("~/k_vij",kvij);          // weight for cost function
	ros::param::get("~/k_cij",kcij);          // weight for cost function
	ros::param::get("~/threshold_grd",threshold); // threshlod for the gradient
	ros::param::get("~/single_test", test);

	obj.designed={0.8,0.3,-0.8,0.3};	// robot1,robot3 position
 	ros::Rate rate(12);


 while(ros::ok()){


  if(obj.pc_ctrl==1 || test==true){
     geometry_msgs::Twist msg;    
     obj.total= obj.total_gradient(obj.designed,kvij,kcij,threshold);
     obj.cmd = obj.vel_calculate(obj.total,motor_lim,desired_v0 ,desired_h); // (0,0)

     msg.linear.x = obj.cmd.linear; msg.angular.z=obj.cmd.angular;
     printf("obj.cmd.linear= %f  obj.cmd.angular= %f\n \n",obj.cmd.linear,obj.cmd.angular);

     obj.pub.publish(msg);   
   }
 rate.sleep();
 ros::spinOnce();

}



return 0;
}
