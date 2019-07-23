
#include <iostream>
#include <tf/tf.h>  
#include "Poseback.h"
#include "path_calculate/heading.h"
#include <cmath>
#define pi 3.1415926


	//pass the num of robot to constuct vector for saving data
 	Poseback::Poseback(int total_num,int this_num):robots(total_num-1){
	m_total_num = total_num ;
	m_label_num = this_num;
	initiallabel(total_num,this_num);
	sub_left_pose = nh.subscribe("/cam1/ar_pose_marker", 1000, &Poseback::pose_getleftCB,this);
	sub_right_pose = nh.subscribe("/cam2/ar_pose_marker", 1000, &Poseback::pose_getrightCB,this);
	multi_udp = nh.subscribe("/cross_info",1000, &Poseback::udp_info,this);
// sending cmd msgs to robot3      
	pub = nh.advertise<geometry_msgs::Twist>("robot3/cmd", 1000); 	
	}
	
	void Poseback::initiallabel(int total,int this_bot){
	    		 
	   for(int i=1;i<(total+1);i++){
		if(i != this_bot)		
		labels.push_back(i);
	 }
	}



////left_cam callack with rotation matrix {cos-75 sin-75;-sin-75 cos-75 }={0.258 -0.965;0.965,0.258}
///{cos-65 sin-65;-sin-65 cos-65 }={0.4226 -0.9063;0.9063,0.4226}
float Poseback::rotation_left(struct robot & rob,double heading, float x,float y){
	float relative_angle;	
	x = 0.9727*x+0.0068*x*x-0.0305*y+0.011; y = 0.8536*y+0.0149*x-0.0294*x*x+0.0427;    // using lsqcurvefitting to conpensate camera
	rob.distancex = 0.4226*x-0.9063*y;
	rob.distancey = 0.9063*x+0.4226*y;
	rob.find_left = 1;
	relative_angle = -heading + (65*0.0174532);
	return relative_angle;
}


////right_cam callback with the rotation matrix {cos75 sin75;-sin75 cos75}={0.258 0.965;-0.906 0.258}
//{cos65 sin65;-sin65 cos65 }={0.4226 0.9063;-0.9063,0.4226}
float Poseback::rotation_right(struct robot & rob,double heading, float x,float y){
	float relative_angle;	
	x = 0.9834*x-0.0003*x*x-0.0428*y-0.0229;  y = 0.8429*y+0.0523*y*y-0.044*x+0.0932;  
	rob.distancex = 0.4226*x+0.9063*y;
	rob.distancey = -0.9063*x+0.4226*y;
	rob.find_right = 1;
	relative_angle = -heading - (65*0.0174532);
	return relative_angle;
}



void Poseback::pose_solve(int rob_num,int i, struct robot & rob, const ar_track_alvar_msgs::AlvarMarkers &req, float (*pf)(struct robot &rob,double heading,float x,float y)){

	float temp_angle; //temp relative angle between j & i
 	if(((req.markers[i].id-1)/4) == (rob_num-1)){
		
// check marker 1 find corrsponding 
          if((req.markers[i].id%4) == 1){
	tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
// return the pose of robot1
      temp_angle = (*pf)(rob,pitch,req.markers[i].pose.pose.position.x, req.markers[i].pose.pose.position.z);
      //benchmark for the marker 
      rob.heading = temp_angle;
  std::cout  <<"relative_heading"<< rob.heading*57.29<<"req.markerid"<<req.markers[i].id<< std::endl;
	  }

	   else if((req.markers[i].id%4) == 2){  //pitch + 1.570

tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
       
        temp_angle = (*pf)(rob,pitch,req.markers[i].pose.pose.position.x, req.markers[i].pose.pose.position.z);
	//computing yaw angle i robot coordination 
	rob.heading = temp_angle+pi/2;
	

	  }
	   else if((req.markers[i].id%4) == 3){ // pitch - 1.570

tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

        temp_angle = (*pf)(rob,pitch,req.markers[i].pose.pose.position.x, req.markers[i].pose.pose.position.z);
	//computing yaw angle i robot coordination (should be modified when pitch < 0)
	rob.heading = temp_angle-pi/2;	
	std::cout  <<"relative_heading"<< rob.heading*57.29<<"req.markerid"<<req.markers[i].id<< std::endl;
	
	  }
	   else{ //pitch +3.141


tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

        temp_angle = (*pf)(rob,pitch,req.markers[i].pose.pose.position.x, req.markers[i].pose.pose.position.z);
	rob.heading = temp_angle+pi;
	//computing yaw angle i robot coordination (should be modified when pitch < 0)
	
	
      
	  }

    }

}



////////////////
//
//  callback for the left camera
//
////////////////



void Poseback::pose_getleftCB(const ar_track_alvar_msgs::AlvarMarkers &msg) {

 	for(int i=0;i<(m_total_num-1);i++)
  	robots[i].find_left = 0;
	


  	if (!msg.markers.empty()) {
    	int mkr_cnt = msg.markers.size();
    

	//check marker from #1 robot (should add bool as flag)

    	for(int i=0;i<mkr_cnt;i++){ 
	for(int j=0;j<(m_total_num-1);j++)
    	pose_solve(labels[j],i,robots[j], msg, rotation_left);

    }


  }



       
}



///////////////////////////////
//
// callback for the right camera
//
//////////////////////////////////





void Poseback::pose_getrightCB(const ar_track_alvar_msgs::AlvarMarkers &msg) {
	
	for(int i=0;i<(m_total_num-1);i++)
         robots[i].find_right = 0;


    	if (!msg.markers.empty()) {

  	int mkr_cnt = msg.markers.size();


	//check marker from #1 robot (should add bool as flag)

    	 for(int i=0;i<mkr_cnt;i++){ // in the future if there are many robots can use break quit for loop when (robot_find=1)

	for(int j=0;j<(m_total_num-1);j++)
        pose_solve(labels[j],i,robots[j], msg, rotation_right);
      

     }

 }
}




///////////////
//   
//  call back for communnication info from outside
//
/////////////////


	void Poseback::udp_info(const path_calculate::heading &msg) 
{
 	 head_self=msg.robot3;
          //robots[0].heading=msg.robot1;
  	//robots[1].heading=msg.robot2; 
	//robots[2].heading=msg.robot4;
  	pc_ctrl=msg.laptop;
	printf("pc_ctrl = %d", pc_ctrl);
}




