#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "control_bot.h"
#include <geometry_msgs/Twist.h>
	float angbound(float ang_velocity,float limitation){
	    if(ang_velocity>limitation)
		return limitation;
	    else 
		return ang_velocity;
        }

int main(int argc, char **argv) {


	ros::init(argc, argv, "cam_pub");
        float desired_v0,motor_lim;  //where desired_h is offset value and should in radain uint
	float desired_h = 0;
	float desired_h2;
	float k_vjm,kvij,kcij,k_rotate,threshold;
	float RS;
	float ang_lim; 
	
	int total_robotn;
	int this_robotn;
	bool test=0;
	bool chang_graph = true;

	ros::param::get("~/desired_v",desired_v0);
	ros::param::get("~/desired_h",desired_h2); // changed desired heading in running
	ros::param::get("~/motor_lim",motor_lim);
	ros::param::get("~/k_vij",kvij);          // weight for cost function
	ros::param::get("~/k_cij",kcij);          // weight for cost function
	ros::param::get("~/threshold_grd",threshold); // threshlod for the gradient
        ros::param::get("~/coverage",RS);
	ros::param::get("~/k_vjm",k_vjm);	// weight for cost function vij_more factor
	ros::param::get("~/single_test", test);
	ros::param::get("~/k_rotate", k_rotate);  
	ros::param::get("~/ang_lim", ang_lim);
	
	ros::param::get("~/total_robotn", total_robotn);
	ros::param::get("~/this_robotn", this_robotn);  
	
	Ctrl_bot robots(total_robotn,this_robotn); 
	
	//Ctrl_bot::dsr_pos p1 = {-1.4,-0.4} ;
	//Ctrl_bot::dsr_pos p2 = {-0.7,0.0};
	//Ctrl_bot::dsr_pos p4 = {0.7,-0.4};
	//robots.graph.push_back(p1);
	//robots.graph.push_back(p2);
	//robots.graph.push_back(p4);

	Ctrl_bot::dsr_pos p1 = {0.6,0.5};
	Ctrl_bot::dsr_pos p2 = {1.2,0.0};
	Ctrl_bot::dsr_pos p4 = {1.8,0.5};
	robots.graph.push_back(p1);
	robots.graph.push_back(p2);
	robots.graph.push_back(p4);

	
	
         // weight for angular velocity

		// struct in gradient robot1,robot2 position


 	ros::Rate rate(12);
   

 while(ros::ok()){


  if(robots.pc_ctrl==1 || robots.pc_ctrl==2 || test==true){
     geometry_msgs::Twist msg;   

	//change the graph shape
	if(robots.pc_ctrl==2 && chang_graph == true){
	desired_h = desired_h2;
	chang_graph == false;
	}

	//the below part can imporve with object-oriented(all calculate in class,but i am lazy to change)
  
     robots.total= robots.total_gradient(kvij,kcij,threshold,RS,k_vjm);
     robots.cmd = robots.vel_calculate(robots.total,motor_lim,desired_v0,desired_h,k_rotate); // (0,0)

     msg.linear.x = robots.cmd.linear; 
     msg.angular.z= angbound(robots.cmd.angular,ang_lim);  // for those nonholonmic robots like(car,differential robots)
     printf("obj.cmd.linear= %f obj.cmd.angular=%f \n \n",robots.cmd.linear,robots.cmd.angular);

     robots.pub.publish(msg);   
   }
 rate.sleep();
 ros::spinOnce();

}



return 0;
}
