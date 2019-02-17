#include "gradient.h"
#include <iostream>
#include <cmath>
#define blind_a 40*pi/180
#define bound 0.5  //safe bound for angle = 60 degree 
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
// below is for numerical calculate the gradient 

//double dd1_x, J_cost,dd1_y, gd_x,gd_y; 
//double den,den_x,den_y;

//J_cost = (distancex-desire_x)*(distancex-desire_x)+(distancey-desire_y)*(distancey-desire_y);
//den = std::sqrt(distancex*distancex+distancey*distancey);
//J_cost /= ((coverage_Rs-den)*(coverage_Rs-den));

//std::cout<< J_cost << std::endl;

//dd1_x = distancex + 0.000000001;
//dd1_y = distancey + 0.000000001;

//gd_x = (dd1_x-desire_x)*(dd1_x-desire_x)+(distancey-desire_y)*(distancey-desire_y);
//den_x = std::sqrt(dd1_x*dd1_x+distancey*distancey);
//gd_x /= (coverage_Rs-den_x)*(coverage_Rs-den_x);

//gd_y = (dd1_y-desire_y)*(dd1_y-desire_y)+(distancex-desire_x)*(distancex-desire_x);
//den_y = std::sqrt(dd1_y*dd1_y+distancex*distancex);
//gd_y /= (coverage_Rs-den_y)*(coverage_Rs-den_y);
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
//printf("cos_ij=%3.2f theat_ij=%3.2f\n",cos_ij,acos(cos_ij)*57.6);

if(cos_ij>bound)
{
double h_cos,d_rijx,d_rijy;


h_cos=(bound-cos_ij)/((cos_a-cos_ij)*(cos_a-cos_ij));
//h_cos=(bound-cos_ij)/((cos_a-cos_ij)*(cos_a-cos_ij)*(cos_a-cos_ij));

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



gradient::grad gradient::total_gradient(const struct dsr_pos &designed,float kv,float kc,float threshold,float RS,float k_vjm){

grad total ={};

printf("robot1.find_left = %d , robot1.find_right= %d \n",robot1.find_left,robot1.find_right);

if((robot1.find_left == 1) ||(robot1.find_right == 1) )
{	
	float temp1_x,temp1_y;

 std::cout<<"robot1.markerID"<<robot1.info_marker<<std::endl;
	l2g_rotation(robot1.distancex,robot1.distancey, head_self,temp1_x,temp1_y); // for the rotation the posion of the robot from local to global
	std::cout<<"heading_rotate"<<57.2974*head_self<<"robot1.x-globle"<<temp1_x<<"robot1.y-goble"<<temp1_y<<std::endl;
	grad vij_1={};grad cij_1={};
	float norm1_R,norm1_D;

	// the variable for norm_R calculateed by robot(defined in class)
	norm1_R = temp1_x*temp1_x+temp1_y*temp1_y;
	// the variable defined inside the function	
	norm1_D = designed.desire_1x*designed.desire_1x+designed.desire_1y*designed.desire_1y;
	
	
	
	if(norm1_R<norm1_D)
	{
	vij_1=gd_vijless(designed.desire_1x,designed.desire_1y,temp1_x,temp1_y);
	// cij should be modify
	cij_1=gd_cij(robot1.heading,temp1_x,temp1_y);
	
	//std::cout<<"vij_1(x) outside"<<vij_1.gx<<vij_1.gy<<std::endl;
	//std::cout<<"using vijless"<<std::endl;
	}

	

	else{
	vij_1=gd_vijmore(designed.desire_1x, designed.desire_1y,temp1_x,temp1_y, RS);
	vij_1.gx*=k_vjm;
	vij_1.gy*=k_vjm;
	cij_1 = gd_cij(robot1.heading,temp1_x,temp1_y);
	//std::cout<<"using vijMORE"<<std::endl;	

	}
	total.gx = 3*kv*vij_1.gx+kc*cij_1.gx;
	total.gy = 3*kv*vij_1.gy+kc*cij_1.gy;



std::cout<<"cij_1(x)="<<cij_1.gx<<std::endl;
std::cout<<"cij_1(Y)="<<cij_1.gy<<std::endl;
std::cout<<"vij_1(x)="<<vij_1.gx<<std::endl;
std::cout<<"vij_1(Y)="<<vij_1.gy<<std::endl;

}

if((robot2.find_left == 1) || (robot2.find_right == 1))
{	
	float temp2_x,temp2_y;
	l2g_rotation(robot2.distancex,robot2.distancey, head_self,temp2_x,temp2_y);
   
	  // for the rotation the posion of the robot from local to global
	grad vij_2={};grad cij_2={};
	float norm2_R,norm2_D;
	// the variable for norm_R calculateed by robot(defined in class)
	norm2_R = temp2_x*temp2_x+temp2_y*temp2_y;
	// the variable defined inside the function	
	norm2_D = designed.desire_2x*designed.desire_2x+designed.desire_2y*designed.desire_2y;

	if(norm2_R<norm2_D){
	vij_2=gd_vijless(designed.desire_2x,designed.desire_2y,temp2_x,temp2_y);
	vij_2.gx*=k_vjm;
	vij_2.gy*=k_vjm;
	cij_2=gd_cij(robot2.heading,temp2_x,temp2_y);
	
	}

	else{
	vij_2=gd_vijmore(designed.desire_2x,designed.desire_2y,temp2_x,temp2_y,RS);
	cij_2=gd_cij(robot2.heading,temp2_x,temp2_y);
	std::cout<<"vij_22(x)="<<vij_2.gx<<std::endl;
	}

	total.gx += 0.5*kv*vij_2.gx+kc*cij_2.gx;
	total.gy += 0.5*kv*vij_2.gy+kc*cij_2.gy;

	std::cout<<"cij_2(x)="<<cij_2.gx<<std::endl;
	std::cout<<"cij_2(Y)="<<cij_2.gy<<std::endl;
	std::cout<<"vij_2(x)="<<vij_2.gx<<std::endl;
	std::cout<<"vij_2(Y)="<<vij_2.gy<<std::endl;
//std::cout<<"norm_Real = "<<norm2_R<<"norm_Design="<<norm2_D<<std::endl;

}
	if (std::sqrt(total.gx*total.gx+total.gy*total.gy)<= threshold)
	{
		total.gx = 0;
		total.gy = 0;
	}

return total;

}
