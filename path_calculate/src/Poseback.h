#ifndef _POSEBACK_H
#define _POSEBACK_H

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "path_calculate/heading.h"
#include <geometry_msgs/Twist.h>

class Poseback{
  public:
	int pc_ctrl; 
        void pose_getleftCB(const ar_track_alvar_msgs::AlvarMarkers& msg);	
	void pose_getrightCB(const ar_track_alvar_msgs::AlvarMarkers& msg);
	void udp_info(const path_calculate::heading &msg); 	
	
	ros::Publisher pub;

	Poseback(){

	sub_left_pose = nh.subscribe("/camera1/ar_pose_marker", 1000, &Poseback::pose_getleftCB,this);
	sub_right_pose = nh.subscribe("/camera2/ar_pose_marker", 1000, &Poseback::pose_getrightCB,this);
	multi_udp = nh.subscribe("/cross_info",1000, &Poseback::udp_info,this);
// sending cmd msgs to robot3      
	pub = nh.advertise<geometry_msgs::Twist>("robot3/cmd", 1000); 
	robot1.find_left=0;
	robot1.find_right=0;
	robot3.find_left=0;
	robot3.find_right=0;
	count_1=0;
	count_2=0;	
	}
	

  protected: 
       
//using struct to save info for ith robot

    struct robot
    {
	float distancex;
	float distancey;
	float heading;        //that is a relative heading for local robot, should change use robotj-roboti
	bool find_left;	     // flag for find robot (1 for find, 0 for not)
	bool find_right;
           
    };	
	

	struct robot robot1;
	struct robot robot3;

	float head_self; 
              // pass the flag for the robots (1=run, 0=2=stop)



 private:
	ros::Subscriber multi_udp;
        ros::Subscriber sub_left_pose;   
	ros::Subscriber sub_right_pose;
	ros::NodeHandle nh;
	int count_1;
	int count_2;


	void pose_solve(int rob_num,int i, struct robot &rob,const ar_track_alvar_msgs::AlvarMarkers &req, void (*pf)(struct robot &rob,float x,float y)); // function for pose analysis, input: robot#,ar_pose,tf_matraix,output: bool for robot detection

	static void rotation_left(struct robot &rob, float x,float y);  // rotation for left camera
	static void rotation_right(struct robot &rob,float x, float y); // rotation for right camera
	
		
	
};

#endif
