#ifndef _CONTROL_BOT_H
#define _CONTROL_BOT_H

#include "gradient.h"

class Ctrl_bot:public gradient
{


public:
 	Ctrl_bot(int a,int b):gradient(a,b){}
 

	struct speed{

		float linear;
		float angular;
	
	};
// input total gradient,v0(designed speed), motor constrain, desired fight direction. OUTPUT: angular_velocity and linear_velocity
// the desired heading direction based on the begin angular offset


	 struct dsr_pos	graph_rotate(struct dsr_pos position, float direction);
	struct speed vel_calculate(struct grad total,const float motor_lim, float v0,float desired_h, float k_rotate);

speed cmd;


private:

	int Sign(const float x);




};

#endif
