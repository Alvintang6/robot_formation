
#include <iostream>
#include <tf/tf.h>  
#include "Poseback.h"
#include "path_calculate/heading.h"
#include <cmath>
#define pi 3.1415926


////left_cam callack with rotation matrix {cos-75 sin-75;-sin-75 cos-75 }={0.258 -0.965;0.965,0.258}
///{cos-65 sin-65;-sin-65 cos-65 }={0.4226 -0.9063;0.9063,0.4226}
void Poseback::rotation_left(struct robot & rob, float x,float y,int i){
	x = 0.9573*x+0.0028*x*x-0.0639*y-0.0007; y = 0.8533*y+0.0686*x-0.0447*x*x+0.0941;    // using lsqcurvefitting to conpensate camera
	rob.distancex = 0.4226*x-0.9063*y;
	rob.distancey = 0.9063*x+0.4226*y;
	rob.find_left = 1;
	rob.info_marker = i;
}


////right_cam callback with the rotation matrix {cos75 sin75;-sin75 cos75}={0.258 0.965;-0.906 0.258}
//{cos65 sin65;-sin65 cos65 }={0.4226 0.9063;-0.9063,0.4226}
void Poseback::rotation_right(struct robot & rob, float x,float y,int i){
	 x = 0.9589*x+0.0029*x*x+0.0074*y-0.0104;  y = 0.8264*y-0.0087*y*y-0.0473*x+0.0985;  
	rob.distancex = 0.4226*x+0.9063*y;
	rob.distancey = -0.9063*x+0.4226*y;
	rob.find_right = 1;
	rob.info_marker = i;
}



void Poseback::pose_solve(int rob_num,int i, struct robot & rob, const ar_track_alvar_msgs::AlvarMarkers &req, void (*pf)(struct robot &rob,float x,float y,int)){


 	if(((req.markers[i].id-1)/4) == (rob_num-1)){
		
// check marker 1 find corrsponding 
          if((req.markers[i].id%4) == 1){
	tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
// return the pose of robot1
      (*pf)(rob,req.markers[i].pose.pose.position.x, req.markers[i].pose.pose.position.z, req.markers[i].id);
      //rob.heading = pitch*(-1) + 1.308 ; 
	  }

	   else if((req.markers[i].id%4) == 2){  //pitch + 1.570

tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
       
        (*pf)(rob,req.markers[i].pose.pose.position.x, req.markers[i].pose.pose.position.z, req.markers[i].id);
	//computing yaw angle i robot coordination (should be modified when pitch < 0)
	//rob.heading = pitch*(-1) + 1.308 + 1.570;
	

	  }
	   else if((req.markers[i].id%4) == 3){ // pitch - 1.570

tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

        (*pf)(rob,req.markers[i].pose.pose.position.x, req.markers[i].pose.pose.position.z, req.markers[i].id);
	//computing yaw angle i robot coordination (should be modified when pitch < 0)
	//rob.heading = pitch*(-1) + 1.308 + 3.141;
	
	  }
	   else{ //pitch +3.141


tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

        (*pf)(rob,req.markers[i].pose.pose.position.x, req.markers[i].pose.pose.position.z, req.markers[i].id);
	//computing yaw angle i robot coordination (should be modified when pitch < 0)
	//rob.heading = pitch*(-1) + 1.308 + 3.141;

      
	  }

    }

}



////////////////
//
//  callback for the left camera
//
////////////////



void Poseback::pose_getleftCB(const ar_track_alvar_msgs::AlvarMarkers &msg) {

  robot1.find_left = 0; robot2.find_left = 0;

  if (!msg.markers.empty()) {
    int mkr_cnt = msg.markers.size();
    

    int i;
//check marker from #1 robot (should add bool as flag)
    for(i=0;i<mkr_cnt;i++){ // in the future if there are many robots can use break quit for loop when (robot_find=1)

    pose_solve(2,i,robot1, msg, rotation_left);


    }

//check marker from #2 robot (should add bool as flag)
    for(i=0;i<mkr_cnt;i++){
     
     pose_solve(3,i,robot2, msg, rotation_left);

    }


  }



       
}



///////////////////////////////
//
// callback for the right camera
//
//////////////////////////////////





void Poseback::pose_getrightCB(const ar_track_alvar_msgs::AlvarMarkers &msg) {

         robot1.find_right = 0; robot2.find_right = 0;


    if (!msg.markers.empty()) {

   int mkr_cnt = msg.markers.size();

     int i;
//check marker from #1 robot (should add bool as flag)
     for(i=0;i<mkr_cnt;i++){ // in the future if there are many robots can use break quit for loop when (robot_find=1)

    pose_solve(2,i,robot1, msg, rotation_right);


     }

//check marker from #2 robot (should add bool as flag)
     for(i=0;i<mkr_cnt;i++){
     
     pose_solve(3,i,robot2, msg, rotation_right);

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
  head_self=msg.robot1;
  robot1.heading=msg.robot2;
  robot2.heading=msg.robot3; 
  pc_ctrl=msg.laptop;
printf("pc_ctrl = %d", pc_ctrl);

}




