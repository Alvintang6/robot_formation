#ifndef _GRADIENT_H
#define _GRADIENT_H

#include "Poseback.h"

class gradient: public Poseback
{
public:

	gradient(int a,int b):Poseback(a,b){}	

	struct grad{
		float gx;
		float gy;
	};


	// this struct should be init in (main class or pass from outside)
	typedef struct dsr_pos{
	  float desire_x;
	  float desire_y;

	}Dsr_pos;


	//the desired_x,y and distance is for each j robot

	std::vector<Dsr_pos> graph;



	struct grad gd_vijless(float desire_x,float desire_y,float distancex, float distancey);
	struct grad gd_vijmore(float desire_x,float desire_y,float distancex, float distancey,float coverage_Rs); 
	struct grad gd_cij(float heading, float disx, float disy);
	struct grad gd_add(int buffer_i,float kv,float kc,float threshold, float RS,float k_vjm);


	struct grad total_gradient(float kv_,float kc_,float threshold_, float RS_,float k_vjm_); 

	grad total;


private:

	void l2g_rotation(const float & x,const float & y,const float & heading, float heading_self,float & rt_x, float & rt_y,float & rt_h);


};

#endif
