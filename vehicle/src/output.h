#ifndef _GRADIENT_H
#define _GRADIENT_H

#include "Poseback.h"

class output: public Poseback
{
public:

	output(int a,int b):Poseback(a,b){}	

	struct position{
		float x_position;
		float y_position;
	};
	
	struct position pose_out();

	position robot4;


private:

	void l2g_rotation(const float & x,const float & y, float heading_self,float & rt_x, float & rt_y);


};

#endif
