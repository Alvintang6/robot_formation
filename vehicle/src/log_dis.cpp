#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "output.h"
#include "vehicle/position.h"
//#include <geometry_msgs/Twist.h>


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
	
	output robot(total_robotn,this_robotn); 
	
	
	
         // weight for angular velocity

		// struct in gradient robot1,robot2 position


 	ros::Rate rate(12);
   

 while(ros::ok()){


  if(robot.pc_ctrl==1 || test==true){
        vehicle::position msg;
	robot.robot4=robot.pose_out();
	//the below part can imporve with object-oriented(all calculate in class,but i am lazy to change)
  
	msg.x_position = robot.robot4.x_position;
	msg.y_position = robot.robot4.y_position;
	msg.heading_self = robot.head_self;

  if(robot.robot4.x_position !=0 || robot.robot4.y_position !=0)
     robot.pub.publish(msg);   
   }
 rate.sleep();
 ros::spinOnce();

}



return 0;
}
