#ifndef _POSEBACK_H
#define _POSEBACK_H

#include <ros/ros.h>
#include <vector>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "path_calculate/heading.h"
#include <geometry_msgs/Twist.h>
#include "vehicle/position.h"

class Poseback{
  public:
	int pc_ctrl; // pass the flag for the robots (1=run, 0=2=stop)
        void pose_getleftCB(const ar_track_alvar_msgs::AlvarMarkers& msg);	
	void pose_getrightCB(const ar_track_alvar_msgs::AlvarMarkers& msg);
	void udp_info(const path_calculate::heading &msg); 
		
	Poseback(int total_num,int this_num);

	ros::Publisher pub;	
	float head_self; 

  protected: 
       
//using struct to save info for ith robot


    typedef struct robot
    {
	float distancex;
	float distancey;
	float heading;        //that is a relative heading for local robot, should change use robotj-roboti
	bool find_left;	     // flag for find robot (1 for find, 0 for not)
	bool find_right;
        int info_marker;   
    }ROBOT;	

	std::vector<int> labels;       // save robots label number
	std::vector<ROBOT> robots;     // save robots infomation
	//struct robot robot1,robot2;
	int m_label_num;
	int m_total_num;


	
        
	void initiallabel(int total,int this_bot);      


 private:
	ros::Subscriber multi_udp;
        ros::Subscriber sub_left_pose;   
	ros::Subscriber sub_right_pose;
	ros::NodeHandle nh;

	//int count_1;
	//int count_2;


	void pose_solve(int rob_num,int i, struct robot &rob,const ar_track_alvar_msgs::AlvarMarkers &req, void (*pf)(struct robot &rob,float x,float y,int i)); // function for pose analysis, input: robot#,ar_pose,tf_matraix,output: bool for robot detection

	static void rotation_left(struct robot &rob, float x,float y, int i);  // rotation for left camera
	static void rotation_right(struct robot &rob,float x, float y, int i); // rotation for right camera
	
		
	
};

#endif
