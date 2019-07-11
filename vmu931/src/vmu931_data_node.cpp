#include "vmu931.h"

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h> //printf
#include <stdlib.h> //exit

void usage(char **argv);

const int calib_simple=1500;
const int calib_inital=300;
const int DATA_SIZE=10; 
//const int MAX_READS=1000;

	


int main(int argc, char **argv)
{     
	ros::init(argc, argv, "vmu931_data"); 
	ros::NodeHandle nh;

	ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/imu/data",1000);
	//ros::Publisher pub_mag = nh.advertise<sensor_msgs::MagneticField>("/imu/mag",1000);

	struct vmu *vmu=NULL;
	// suppose we are intersted in euler, magnetometer and accelerometer
	// note that quaternions and heading use different data types
	struct vmu_txyz euler_data[DATA_SIZE];	
	struct vmu_txyz gyro_data[DATA_SIZE];

	struct vmu_data data={0}; //the library will return data here and note number of read values in data.size
	struct vmu_size size={0}; //this notes sizes of our arrays, data.size is refreshed with this value before read

	size.euler = size.gyro = DATA_SIZE;	

	data.gyro = gyro_data;
	data.euler = euler_data;
	data.size = size;


	int gyro_count = 0;
	
    	
	double gyro_inital_x=0;
	double gyro_inital_y=0;
	double gyro_inital_z=0;
	double gyro_bias_x=0;
	double gyro_bias_y=0;
	double gyro_bias_z=0;
	double gyro_cur_x=0;
	double gyro_cur_y=0;
	double gyro_cur_z=0;	
	
	


	tf::Quaternion temp;

	std::string tty_port="/dev/ttyACM0";
	ros::param::get("~port",tty_port);
	
	const char *tty_device=tty_port.c_str();

	int ret, reads=0;
	
	
	
	if( (vmu=vmu_init(tty_device)) == NULL)
	{
		perror(	"unable to initialize VMU931\n\n"
				"hints:\n"
				"- it takes a few seconds after plugging in to initialize device\n"
				"- make sure you are using correct tty device (dmesg after plugging vmu)\n"
				"- if all else fails unplug/plug VMU931\n\n"
				"error details");
		return 1;
	}	
	
	if( vmu_stream(vmu,VMU_STREAM_EULER |VMU_STREAM_GYRO) == VMU_ERROR )
	{
		ROS_INFO("failed to stream euler/mag/accel data");
		exit(EXIT_FAILURE);
}
		


 int calib_offset=calib_simple+calib_inital;


        ros::Rate rate(150);
        
while( ((ret=vmu_read_all(vmu, &data)) != VMU_ERROR))
{
	double gyro_temp_x=0;
	double gyro_temp_y=0;
	double gyro_temp_z=0;


	    	//sensor_msgs::MagneticField mag_data;
		//mag_data.header.stamp = ros::Time::now();
		//mag_data.header.frame_id = "imu_link";

	        sensor_msgs::Imu vmu_data;
    		vmu_data.header.stamp = ros::Time::now();
    		vmu_data.header.frame_id = "imu_link";

	for(int i=0;i<data.size.gyro;++i){
    //should add calibration bias to  received angular velocity and pass to to corrected IMU data object
		vmu_data.angular_velocity.x = data.gyro[i].x*0.01745329;
  		vmu_data.angular_velocity.y = data.gyro[i].y*0.01745329;
  		vmu_data.angular_velocity.z = data.gyro[i].z*0.01745329;
	}


	for(int i=0;i<data.size.euler;++i){
		gyro_temp_x = data.euler[i].x;
		gyro_temp_y = data.euler[i].y;
		gyro_temp_z = data.euler[i].z;
	  
	}
		


	if(gyro_count==calib_inital)
	{		
	ROS_INFO("gyro calibrating... please donot move & wait");
	
			gyro_inital_x = gyro_temp_x;
		        gyro_inital_y = gyro_temp_y;
			gyro_inital_z = gyro_temp_z;	
	  
	}

	if(gyro_count==calib_offset){
		

			gyro_bias_x= (gyro_temp_x-gyro_inital_x)/calib_simple;
		        gyro_bias_y= (gyro_temp_y-gyro_inital_y)/calib_simple;
			gyro_bias_z= (gyro_temp_z-gyro_inital_z)/calib_simple;
			gyro_inital_x = gyro_temp_x;
		        gyro_inital_y = gyro_temp_y;
			gyro_inital_z = gyro_temp_z;
	
	  ROS_WARN("gyro calibrated with BIAS_Z %f",gyro_bias_z);
	}

	if(gyro_count>calib_offset){
		for(int i=0;i<data.size.euler;++i){
			gyro_cur_x = gyro_temp_x-gyro_inital_x-(gyro_bias_x*(gyro_count-calib_offset));
			gyro_cur_x *=0.01745329; //degree to rad unit
		        gyro_cur_y = gyro_temp_y-gyro_inital_y-(gyro_bias_y*(gyro_count-calib_offset));
			gyro_cur_y *=0.01745329;
			gyro_cur_z = gyro_temp_z-gyro_inital_z-(gyro_bias_z*(gyro_count-calib_offset));
			gyro_cur_z *=0.01745329;
	  
		}

		temp.setRPY(gyro_cur_x,gyro_cur_y,gyro_cur_z); 
		vmu_data.orientation.x = temp[0];
		vmu_data.orientation.y = temp[1];
		vmu_data.orientation.z = temp[2];
		vmu_data.orientation.w = temp[3];
	}

	gyro_count++;
	
		
	//refresh the sizes of the arrays for data streams
	pub_imu.publish(vmu_data);

  	
	     data.size = size;

	
	
		//break;
	rate.sleep();
			
}//end while

		







	if(ret == VMU_ERROR)
		perror("failed to read from VMU931");
	else
		printf("success reading from VMU931, bye...\n");
	
	vmu_close(vmu);
	
	return 0;
}




