#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "calib.h"







int main(int argc, char **argv) {


	ros::init(argc, argv, "arlistener");
	Calib obj; 
 
		

 	ros::Rate rate(20);


 while(ros::ok()){


    
    //obj.pub.publish(msg);
    rate.sleep();
   
    ros::spinOnce();
    
 }

return 0;
}
