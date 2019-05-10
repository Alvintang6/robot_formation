#include "gradient.h"
#include <iostream>
#include <cmath> 
#define blind_a 40*pi/180
#define bound 0.5  // bound for angle = 60 degree 
#define pi 3.1415926




void gradient::l2g_rotation(const float &x,const float & y, float heading_self, float &rt_x,float &rt_y){

	rt_x = x*cos(heading_self)-y*sin(heading_self);
	rt_y = sin(heading_self)*x+cos(heading_self)*y;	


}


gradient::grad gradient::gd_vijless(float desire_x,float desire_y,float distancex, float distancey){

// this cost function is for the formation less Hij;
// using dj/dxi = dj/d(xi-xj)*d(xi-xj)/dxi= dj/d(xi-xj);
struct grad vij;
double gx,gy;

gx = -1*(2*distancex*((desire_x-distancex)*(desire_x-distancex)+(desire_y-distancey)*(desire_y-distancey)));
gx /= (distancex*distancex+distancey*distancey)*(distancex*distancex+distancey*distancey); 
gx -=(2*desire_x-2*distancex)/(distancex*distancex+distancey*distancey);
vij.gx = gx; 

gy = -1*(2*distancey*((desire_y-distancey)*(desire_y-distancey)+(desire_x-distancex)*(desire_x-distancex)));
gy /= (distancey*distancey+distancex*distancex)*(distancey*distancey+distancex*distancex); 
gy -=(2*desire_y-2*distancey)/(distancey*distancey+distancex*distancex);
vij.gy = gy;


return vij;


}



gradient::grad gradient::gd_vijmore(float desire_x,float desire_y,float distancex, float distancey,float coverage_Rs){
// this gradient is for the current formation large than Hij, note part using numerical way.

struct grad vij;

double gx,gy;

gx = (2*distancex*((desire_x-distancex)*(desire_x-distancex)+(desire_y-distancey)*(desire_y-distancey)));
gx /= (std::sqrt(distancex*distancex+distancey*distancey)*(pow((coverage_Rs-std::sqrt(distancex*distancex+distancey*distancey)),3)));
gx -= (2*desire_x-2*distancex)/pow((coverage_Rs-sqrt(distancex*distancex+distancey*distancey)),2);

gy = (2*distancey*((desire_x-distancex)*(desire_x-distancex)+(desire_y-distancey)*(desire_y-distancey)));
gy /= (std::sqrt(distancex*distancex+distancey*distancey)*(pow((coverage_Rs-std::sqrt(distancex*distancex+distancey*distancey)),3)));
gy -= (2*desire_y-2*distancey)/pow((coverage_Rs-sqrt(distancex*distancex+distancey*distancey)),2);


if(distancex*distancex+distancey*distancey< coverage_Rs* coverage_Rs)
	{vij.gx = gx;
	vij.gy = gy;
}
else {
	printf("excecced the vision area \n");
	vij.gx = -gx;
	vij.gy = -gy;

}


std::cout<<"vij_more"<<vij.gx<<vij.gy<<std::endl;

return vij;

}


////////////////
// this function is used for calculate Cij (make robot out of blind zone)
//
// bound should be = cos(thet_Er)
//
//////////////////
gradient::grad gradient::gd_cij(float heading,float disx, float disy){

struct grad cij;


double cos_ij;
double dd1_x,dd1_y, gd_x,gd_y;
double H_cost,ddcos_xij,ddcos_yij;
float cos_heading = cos(heading), sin_heading = sin(heading), cos_a = cos(blind_a) ;

cos_ij = -sin_heading*disx+cos_heading*disy;
cos_ij /= std::sqrt((sin_heading*sin_heading+cos_heading*cos_heading)*(disx*disx+disy*disy));


//printf("heading = %f \n", heading);
//printf("cos_ij=%3.2f theat_ij=%3.2f\n",cos_ij,acos(cos_ij));

if(cos_ij>bound)
{
double h_cos,d_rijx,d_rijy;

h_cos=(bound-cos_ij)/((cos_a-cos_ij)*(cos_a-cos_ij));

d_rijx=(sin_heading/pow(((cos_heading*cos_heading + sin_heading*sin_heading)*(disx*disx + disy*disy)),0.5) + (disx*(disy*cos_heading - disx*sin_heading)*(cos_heading*cos_heading + sin_heading*sin_heading))/pow(((cos_heading*cos_heading + sin_heading*sin_heading)*(disx*disx + disy*disy)),1.5))/pow((1 - (disy*cos_heading - disx*sin_heading)*(disy*cos_heading - disx*sin_heading)/((cos_heading*cos_heading + sin_heading*sin_heading)*(disx*disx + disy*disy))),0.5);


d_rijy=-(cos_heading/pow(((cos_heading*cos_heading + sin_heading*sin_heading)*(disx*disx + disy*disy)),0.5) - (disy*(disy*cos_heading - disx*sin_heading)*(cos_heading*cos_heading + sin_heading*sin_heading))/pow(((cos_heading*cos_heading + sin_heading*sin_heading)*(disx*disx + disy*disy)),1.5))/(pow(1 - (disy*cos_heading - disx*sin_heading)*(disy*cos_heading - disx*sin_heading)/(((cos_heading*cos_heading + sin_heading*sin_heading)*(disx*disx + disy*disy))),0.5));

cij.gx=h_cos*d_rijx;
cij.gy=h_cos*d_rijy;


}

// should also have the if(){} for cosij>cos(blind_a)
else{
cij.gx = 0;
cij.gy = 0;
}

return cij;

}




gradient::grad gradient::gd_add(int buffer_i,float kv,float kc,float threshold, float RS,float k_vjm){
	grad return_grad_={0,0};

	if((robots[buffer_i].find_left == 1) ||(robots[buffer_i].find_right == 1) )
	{	
		float temp1_x,temp1_y;

 		std::cout<<"robot1.markerID"<<robots[0].info_marker<<std::endl;
		// for the rotation the posion of the robot from local to global
		l2g_rotation(robots[buffer_i].distancex,robots[buffer_i].distancey, head_self,temp1_x,temp1_y); 
		std::cout<<"heading_rotate"<<57.2974*head_self<<"robot1.x-globle"<<temp1_x<<"robot1.y-goble"<<temp1_y<<std::endl;
		grad vij_={};grad cij_={};
		float norm1_R,norm1_D;

		// the variable for norm_R calculateed by robot(defined in class)
		norm1_R = temp1_x*temp1_x+temp1_y*temp1_y;
		// the variable defined inside the function	
		norm1_D = graph[buffer_i].desire_x*graph[buffer_i].desire_x+graph[buffer_i].desire_y*graph[buffer_i].desire_y;
	
	
		//std::cout<<"norm1_Real = "<<norm1_R<<"norm1_Design"<<norm1_D<<std::endl;

	
		if(norm1_R<norm1_D)
		{
			vij_=gd_vijless(graph[buffer_i].desire_x,graph[buffer_i].desire_y,temp1_x,temp1_y);
			// cij should be modify
			cij_=gd_cij(robots[buffer_i].heading,temp1_x,temp1_y);
	
			//std::cout<<"vij_1(x) outside"<<vij_1.gx<<vij_1.gy<<std::endl;
			//std::cout<<"using vijless"<<std::endl;
		}
	
		else{
			vij_=gd_vijmore(graph[buffer_i].desire_x, graph[buffer_i].desire_y,temp1_x,temp1_y, RS);
			vij_.gx*=k_vjm;
			vij_.gy*=k_vjm;	
			cij_ = gd_cij(robots[buffer_i].heading,temp1_x,temp1_y);
			//std::cout<<"using vijMORE"<<std::endl;	

		}
		return_grad_.gx = kv*vij_.gx+kc*cij_.gx;
		return_grad_.gy = kv*vij_.gy+kc*cij_.gy;

	}
		

	return return_grad_;
}




gradient::grad gradient::total_gradient(float kv_,float kc_,float threshold_,float RS_,float k_vjm_){

	grad temp;
	grad total ={0,0};

	//printf("robot1.find_left = %d , robot1.find_right= %d \n",robots[0].find_left,robots[0].find_right);

	
	for(int i=0;i<(m_total_num-1);i++){
	// can change to operator overload
	temp.gx=0;
	temp.gy=0;
	temp = gd_add(i,kv_,kc_,threshold_,RS_,k_vjm_);
	total.gx += temp.gx;
	total.gy += temp.gy;	

	}


	
	if (std::sqrt(total.gx*total.gx+total.gy*total.gy)<= threshold_)
	{
		total.gx = 0;
		total.gy = 0;
	}

return total;

}
