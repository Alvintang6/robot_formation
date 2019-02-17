#ifndef _GRADIENT_H
#define _GRADIENT_H

#include "Poseback.h"

class gradient: public Poseback
{
public:



//void show_gradient();  //function for testing


	struct grad{
		float gx;
		float gy;
	};


// this struct should be init in (main class or pass from outside)
	struct dsr_pos{
		float desire_1x,desire_1y;
		float desire_2x,desire_2y;
	};

dsr_pos designed;

//the desired_x,y and distance is for each j robot
//

	struct grad gd_vijless(float desire_x,float desire_y,float distancex, float distancey);

	struct grad gd_vijmore(float desire_x,float desire_y,float distancex, float distancey,float coverage_Rs);

// should using relative angle,(can using thetaj -thetai or get from qr code) 
	struct grad gd_cij(float heading, float disx, float disy);



// should be finished ......
	struct grad total_gradient(const struct dsr_pos &designed,float kv,float kc,float threshold, float RS,float k_vjm); 

	grad total;


private:

	void l2g_rotation(const float & x,const float & y, float heading_self,float & rt_x, float & rt_y);


};

#endif
