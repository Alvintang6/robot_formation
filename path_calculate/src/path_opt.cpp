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
	float v_second;
	
	int total_robotn;
	int this_robotn;
	bool test=0;
	bool change_pattern = true;
	bool change_graph = true;
	bool change_v = true;

	ros::param::get("~/desired_v",desired_v0);
	ros::param::get("~/v_second",v_second);
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
	

	Ctrl_bot::dsr_pos p1 = {-1.4,0.0} ;
	Ctrl_bot::dsr_pos p2 = {-0.7,0.4};
	Ctrl_bot::dsr_pos p4 = {0.7,0.4};
	
	Ctrl_bot::dsr_pos p1_2 = {-1.4,-0.4} ;
	Ctrl_bot::dsr_pos p2_2 = {-0.7,0.0};
	Ctrl_bot::dsr_pos p4_2 = {0.7,-0.4};
	robots.graph.push_back(p1);
	robots.graph.push_back(p2);
	robots.graph.push_back(p4);

	
	
         // weight for angular velocity

		// struct in gradient robot1,robot2 position


 	ros::Rate rate(12);
   



 while(ros::ok()){


  if(robots.pc_ctrl==1 ||robots.pc_ctrl==2 || robots.pc_ctrl==3 || robots.pc_ctrl==4 || test==true){
     geometry_msgs::Twist msg;   

	//change the graph shape
	if(robots.pc_ctrl==2 && change_graph == true){
	desired_h = desired_h2;
	change_graph = false;
	for(int i=0;i<(total_robotn-1);i++){
		robots.graph[i] = robots.graph_rotate(robots.graph[i],desired_h);
	} 
	}
	
	if(robots.pc_ctrl==4 && change_pattern == true){
		change_pattern = false;
		robots.graph[0] = p1_2; 
		robots.graph[1] = p2_2; 
		robots.graph[2] = p4_2;	
	}


 	if(robots.pc_ctrl==3 && change_v == true){
	change_v = false;
	desired_v0 = v_second;
	}
	//the below part can imporve with object-oriented(all calculate in class,but i am lazy to change)
  
     robots.total= robots.total_gradient(kvij,kcij,threshold,RS,k_vjm); // in the gradient class
     robots.cmd = robots.vel_calculate(robots.total,motor_lim,desired_v0,desired_h,k_rotate); // in control class (0,0)

     msg.linear.x = robots.cmd.linear/1.41;             // low-level controller model miss a factor of sqrt(2) for omni-car
     msg.angular.z=angbound(robots.cmd.angular,ang_lim);
     printf("obj.cmd.linear= %f obj.cmd.angular=%f \n \n",robots.cmd.linear,robots.cmd.angular);

     robots.pub.publish(msg);   
   }
 rate.sleep();
 ros::spinOnce();

}



return 0;
}
