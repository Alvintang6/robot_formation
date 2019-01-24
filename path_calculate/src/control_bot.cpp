#include "control_bot.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#define pi 3.1415926



int Ctrl_bot::Sign(const float x){
	if(x<0)
	return -1;
	else
	return 1;
}



Ctrl_bot::speed Ctrl_bot::vel_calculate( const struct grad total,const float motor_lim, float v0, float desired_h){


// should consider the converge error when
struct speed cmd;
float Vx, Vy;

	if(fabs(total.gx)<motor_lim && fabs(total.gy)<motor_lim){  //should have a motor lim per-defined . 	
	
	//Vx= total.gx+v0*sin(head_self-desired_h); Vy=total.gy+v0*cos(head_self-desired_h);
	
	//if(Vy > 0)
	//{
	
	
		//cmd.linear = std::sqrt(Vx*Vx +Vy*Vy);
		//cmd.angular = -Vx-sin((head_self-desired_h)/2); //  desired_h can using macro 	def or pass parameter
	//}
	//else{

		//cmd.linear = -std::sqrt(Vx*Vx+Vy*Vy);
		//cmd.angular = Vx-std::sin((head_self-desired_h)/2);

	//}
	
		cmd.linear = v0-std::sqrt(total.gx*total.gx+total.gy*total.gy)*(Sign(total.gx*sin(head_self)-total.gy*cos(head_self)));
		cmd.angular = v0*(-total.gx*sin((head_self+pi+desired_h)*0.5)+total.gy*cos((head_self+pi+desired_h)*0.5))-sin((head_self-desired_h)*0.5);
		
		


}

else{  

//for those gradient which exceed the limtation(using generalized way to bound) 
	double temp_max;
	float norm_x,norm_y; 
	temp_max = fabs(total.gx)>fabs(total.gy) ? fabs(total.gx):fabs(total.gy);


 	norm_x = total.gx/(temp_max/motor_lim);
	float factor = (temp_max/motor_lim);
 	norm_y = total.gy/(temp_max/motor_lim);


		cmd.linear = v0-std::sqrt(norm_x*norm_x+norm_y*norm_y)*(Sign(norm_x*sin(head_self)-norm_y*cos(head_self)));
		cmd.angular = v0*(-norm_x*sin((head_self+pi+desired_h)*0.5)+norm_y*cos((head_self+pi+desired_h)*0.5))-sin((head_self-desired_h)*0.5);

     




	//Vx= norm_x+v0*sin(head_self-desired_h); Vy=norm_y+v0*cos(head_self-desired_h);

	//if(Vy > 0)
	//{
		//cmd.linear = std::sqrt(Vx*Vx+Vy*Vy);
		//cmd.angular = -norm_x-std::sin((head_self-desired_h)/2); //  .desired_h can using macro 	def or pass parameter
	 	
		//}
	//else{

		//cmd.linear = -std::sqrt(Vx*Vx+Vy*Vy);
		//cmd.angular = norm_x-std::sin((head_self-desired_h)/2);
	//}
	
     


}

printf("robot1 position =%1.4f,%1.4f,robot2 position =%1.4f,%1.4f",robot1.distancex,robot1.distancey,robot2.distancex,robot2.distancey);
std::cout<<"total.gx "<<total.gx<<"and total.gy"<<total.gy  <<std::endl;

if (cmd.linear<0.1 && cmd.linear>-0.1)
	cmd.linear=0;

return cmd;

}




