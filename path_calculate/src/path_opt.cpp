#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "control_bot.h"
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv) {


	ros::init(argc, argv, "cam_pub");
        float desired_v0,desired_h,motor_lim;  //where desired_h is offset value and should in radain uint
	float k_vjm,kvij,kcij,k_rotate,threshold;
	float RS;
	int total_robotn;
	int this_robotn;
	bool test=0;
	
	ros::param::get("~/desired_v",desired_v0);
	ros::param::get("~/desired_h",desired_h);
	ros::param::get("~/motor_lim",motor_lim);
	ros::param::get("~/k_vij",kvij);          // weight for cost function
	ros::param::get("~/k_cij",kcij);          // weight for cost function
	ros::param::get("~/threshold_grd",threshold); // threshlod for the gradient
        ros::param::get("~/coverage",RS);
	ros::param::get("~/k_vjm",k_vjm);	// weight for cost function vij_more factor
	ros::param::get("~/single_test", test);
	ros::param::get("~/k_rotate", k_rotate);  
	
	ros::param::get("~/total_robotn", total_robotn);
	ros::param::get("~/this_robotn", this_robotn);  
	
	Ctrl_bot robots(total_robotn,this_robotn); 
	
	Ctrl_bot::dsr_pos p1 = {-1.4,-0.3} ;
	Ctrl_bot::dsr_pos p2 = {-0.7,0};
	Ctrl_bot::dsr_pos p4 = {0.7,-0.3};
	robots.graph.push_back(p1);
	robots.graph.push_back(p2);
	robots.graph.push_back(p4);


	
	
         // weight for angular velocity

		// struct in gradient robot1,robot2 position


 	ros::Rate rate(12);
   

 while(ros::ok()){


  if(robots.pc_ctrl==1 || test==true){
     geometry_msgs::Twist msg;   

	//the below part can imporve with object-oriented(all calculate in class,but i am lazy to change)
  
     robots.total= robots.total_gradient(kvij,kcij,threshold,RS,k_vjm);
     robots.cmd = robots.vel_calculate(robots.total,motor_lim,desired_v0,desired_h,k_rotate); // (0,0)

     msg.linear.x = robots.cmd.linear; msg.angular.z=robots.cmd.angular;
     printf("obj.cmd.linear= %f obj.cmd.angular=%f \n \n",robots.cmd.linear,robots.cmd.angular);

     robots.pub.publish(msg);   
   }
 rate.sleep();
 ros::spinOnce();

}



return 0;
}
