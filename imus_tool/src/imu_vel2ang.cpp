
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>
#include <math.h>


//This node only calculate ang.z and pub by quanter


        const int calib_sample=800;
	const int calib_init = 200;	

        double g_acc_x = 0.0;
        double g_acc_y = 0.0;
        double g_acc_z = 0.0;

        double g_imu_dt = 0.0;
        double g_imu_x = 0.0;
        double g_imu_y = 0.0;
        double g_imu_z = 0.0;





ros::Time g_last_imu_time(0.0);




void quater_CB(const sensor_msgs::Imu& msg){
	///	
	 ros::Time current_time = ros::Time::now();
  

  
    g_imu_x = msg.angular_velocity.x;
    g_imu_y = msg.angular_velocity.y;
    g_imu_z = msg.angular_velocity.z;
    g_acc_x = msg.linear_acceleration.x;
    g_acc_x = msg.linear_acceleration.y;
    g_acc_x = msg.linear_acceleration.z;

  
  


}




int main(int argc, char **argv){

        double gyro_bias_x=0;
        double gyro_bias_y=0;
        double gyro_bias_z=0;
	long int imu_count = 0;
	double theta_x = 0.0;
	double theta_y = 0.0;
	double theta_z = 0.0;

	int calib_offset = calib_init + calib_sample; 
        ros::init(argc, argv, "vel2ang");
        ros::NodeHandle nh;

        ros::Subscriber sub = nh.subscribe("/imu/data_raw", 1000, quater_CB);
	ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/imu/data",1000);
        ros::Rate rate(200);
	
while(nh.ok())
{
	

	ros::spinOnce();
	        sensor_msgs::Imu imu_data;
    		imu_data.header.stamp = ros::Time::now();
    		imu_data.header.frame_id = "imu_link";

	// sample the beginning period as imu bias 
        if(imu_count >= calib_init && imu_count < calib_offset  ){
		gyro_bias_x += g_imu_x;
		gyro_bias_y += g_imu_y;
		gyro_bias_z += g_imu_z;
  		
	}

	


	if(imu_count==calib_init)
	{		
	ROS_INFO("gyro calibrating... please donot move & wait");
			  
	}

	if(imu_count == calib_offset){
		
			g_last_imu_time = ros::Time::now();
			gyro_bias_x = gyro_bias_x/calib_sample;
		        gyro_bias_y = gyro_bias_y/calib_sample;
			gyro_bias_z = gyro_bias_z/calib_sample;			
	
	  ROS_INFO("gyro calibrated with BIAS_Z %f",gyro_bias_z);
	}

	

	if(imu_count>calib_offset){
			ros::Time current_time = ros::Time::now();
			g_imu_dt = (current_time - g_last_imu_time).toSec();
			g_last_imu_time = current_time;
			theta_x = theta_x - (g_imu_x-gyro_bias_x)*g_imu_dt;
			theta_y = theta_y - (g_imu_y-gyro_bias_y)*g_imu_dt;
			theta_z = theta_z - (g_imu_z-gyro_bias_z)*g_imu_dt;
		
	tf::Quaternion temp;
		temp.setRPY(theta_x,theta_y,theta_z); 
		imu_data.orientation.x = temp[0];
		imu_data.orientation.y = temp[1];
		imu_data.orientation.z = temp[2];
		imu_data.orientation.w = temp[3];
		imu_data.angular_velocity.x = g_imu_x-gyro_bias_x;
		imu_data.angular_velocity.y = g_imu_y-gyro_bias_y;
		imu_data.angular_velocity.z = g_imu_z-gyro_bias_z;
	}

	imu_count ++;
	
	imu_data.linear_acceleration.x = g_imu_x;
	imu_data.linear_acceleration.y = g_imu_y;
	imu_data.linear_acceleration.z = g_imu_z;


	//refresh the sizes of the arrays for data streams
	pub_imu.publish(imu_data);

	

	rate.sleep();
			
}//end while




return 0;
 }
