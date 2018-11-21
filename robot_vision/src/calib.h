#ifndef _POSEBACK_H
#define _POSEBACK_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <cstdio>

class Calib{
  public:

        void pose_CB(const ar_track_alvar_msgs::AlvarMarkers& req);
	ros::Publisher pub;	
	bool running();

	Calib(){

	
	ros::NodeHandle nh("~");
	//nh_private.param<std::string>("output_file", output_file_, "imu_calib.yaml");
	calib_ar = nh.subscribe("/ar_pose_marker", 1000, &Calib::pose_CB,this);
        
	}
	

  protected: 
       
//using struct to save info for ith robot



 private:
	

	
	
	

	void pose_solve(int rob_num,int i,const ar_track_alvar_msgs::AlvarMarkers &req); 
	
	
        ros::Subscriber calib_ar;   //should have 2 one for left one for right .
	
	
};

#endif

