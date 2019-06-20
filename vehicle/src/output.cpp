#include "output.h"
#include <iostream>
#include <cmath> 
#define blind_a 40*pi/180
#define bound 0.5  // bound for angle = 60 degree 
#define pi 3.1415926




void output::l2g_rotation(const float &x,const float & y, float heading_self, float &rt_x,float &rt_y){

	rt_x = x*cos(heading_self)-y*sin(heading_self);
	rt_y = sin(heading_self)*x+cos(heading_self)*y;	


}




////////////////
// this function is used for calculate Cij (make robot out of blind zone)
//
// bound should be = cos(thet_Er)
//
//////////////////


output::position output::pose_out() {
	position return_pose = {0,0};
	int robot_num = 2; //robot_num should be real_num-1-this
	float temp1_x,temp1_y;

	if((robots[robot_num].find_left == 1) ||(robots[robot_num].find_right == 1) )
	{	
		

 		std::cout<<"robot1.markerID"<<robots[2].info_marker<<std::endl;
		
		// for the rotation the posion of the robot from local to global
		l2g_rotation(robots[robot_num].distancex,robots[robot_num].distancey, head_self,temp1_x,temp1_y); 
		

		
}
	
	return_pose.x_position=temp1_x;
	return_pose.y_position=temp1_y;


return return_pose;

}
