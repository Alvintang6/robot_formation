
#include "calib.h"

////left_cam callack with rotation matrix {cos-75 -sin-75;sin-75 cos-75 }={0.422 0.906;-0.906,0.422}





void Calib::pose_solve(int rob_num,int i, const ar_track_alvar_msgs::AlvarMarkers &req){


 	if(((req.markers[i].id-1)/4) == (rob_num-1)){
		
// check marker 1 find corrsponding 
          if((req.markers[i].id%4) == 1){
	tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
// return the pose of robot1
      printf("the position of robot%d, is(marker%d) x= %f,y=%f with pitch=%f \n",rob_num,req.markers[i].id,req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.z, pitch);


      //rob.heading = pitch*(-1) + 1.308 ;  
	  }

	   else if((req.markers[i].id%4) == 2){  //pitch + 1.570

	tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
       
       printf("the position of robot%d, is(marker%d) x= %f,y=%f with pitch=%f \n",rob_num,req.markers[i].id,req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.z, pitch);

	//rob.heading = pitch*(-1) + 1.308 + 1.570;

	  }
	   else if((req.markers[i].id%4) == 3){ // pitch - 1.570

tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      printf("the position of robot%d, is(marker%d) x= %f,y=%f with pitch=%f \n",rob_num,req.markers[i].id,req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.z, pitch);

	//computing yaw angle i robot coordination (should be modified when pitch < 0)
	//rob.heading = pitch*(-1) + 1.308 + 3.141;

	  }
	   else{ //pitch +3.141


tf::Quaternion q(req.markers[i].pose.pose.orientation.x, req.markers[i].pose.pose.orientation.y, req.markers[i].pose.pose.orientation.z, req.markers[i].pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

       printf("the position of robot%d, is(marker%d) x= %f,y=%f with pitch=%f \n",rob_num,req.markers[i].id,req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.z, pitch);

	//rob.heading = pitch*(-1) + 1.308 + 3.141;


      
	  }

    }

}






void Calib::pose_CB(const ar_track_alvar_msgs::AlvarMarkers &msg) {


  if (!msg.markers.empty()) {
    int mkr_cnt = msg.markers.size();
    

    int i;
//check marker from #1 robot (should add bool as flag)
    for(i=0;i<mkr_cnt;i++){ // in the future if there are many robots can use break quit for loop when (robot_find=1)

    pose_solve(1,i, msg);

    }

//check marker from #2 robot (should add bool as flag)
    for(i=0;i<mkr_cnt;i++){
     
     pose_solve(2,i, msg);


    }

  }





       
}








