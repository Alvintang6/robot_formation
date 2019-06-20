#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <riki_msgs/Velocities.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

double g_vel_x = 0.0;
double g_vel_y = 0.0;

double g_vel_dt = 0.0;
//double g_imu_dt = 0.0;
double g_imu_z = 0.0;
//imu rotation angle
double g_imu_angle = 0.0;

ros::Time g_last_loop_time(0.0);
ros::Time g_last_vel_time(0.0);
//ros::Time g_last_imu_time(0.0);

void velCallback( const riki_msgs::Velocities& vel) {
  //callback every time the robot's linear velocity is received
  ros::Time current_time = ros::Time::now();

  g_vel_x = vel.linear_x;
  g_vel_y = vel.linear_y;

  g_vel_dt = (current_time - g_last_vel_time).toSec();
  g_last_vel_time = current_time;
}

void IMUCallback( const sensor_msgs::Imu::ConstPtr& imu){
  //callback every time the robot's angular velocity is received
  //ros::Time current_time = ros::Time::now();
  
    g_imu_z = imu->angular_velocity.z;
 // }

  //g_imu_dt = (current_time - g_last_imu_time).toSec();
  //g_last_imu_time = current_time;
	tf::Quaternion q( imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
	tf::Matrix3x3 m(q);
	double roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);
	g_imu_angle=yaw;


}

int main(int argc, char** argv){
  double angular_scale, linear_scale;
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("raw_vel", 50, velCallback);
  ros::Subscriber imu_sub = n.subscribe("imu/data", 50, IMUCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  nh_private_.getParam("angular_scale", angular_scale);
  nh_private_.getParam("linear_scale", linear_scale);

  double rate = 20.0;
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();

    //linear velocity is the linear velocity published from the Teensy board in x axis
    double linear_velocity_x = g_vel_x;

    //linear velocity is the linear velocity published from the Teensy board in y axis
    double linear_velocity_y = g_vel_y;

    //angular velocity is the rotation in Z from imu_filter_madgwick's output
   // double angular_velocity = g_imu_z;

    //calculate angular displacement  θ = ω * t
    //double delta_theta = angular_velocity * g_imu_dt * angular_scale; //radians

	ros::Time current_time = ros::Time::now();
	g_vel_dt = (current_time - g_last_vel_time).toSec();
	g_last_vel_time = current_time;

    double delta_x = (linear_velocity_x * cos(g_imu_angle) - linear_velocity_y * sin(g_imu_angle)) * g_vel_dt *linear_scale; //m
    double delta_y = (linear_velocity_x * sin(g_imu_angle) + linear_velocity_y * cos(g_imu_angle)) * g_vel_dt * linear_scale; //m

    //calculate current position of the robot
    x_pos += delta_x;
    y_pos += delta_y;
    //theta += delta_theta;

    //calculate robot's heading in quarternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x;
    odom.twist.twist.linear.y = linear_velocity_y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from IMU
    odom.twist.twist.angular.z = g_imu_z;

	//TODO: include covariance matrix here
	odom.pose.covariance[0] = 20;
	odom.pose.covariance[7] = 20;
	odom.pose.covariance[14] = FLT_MAX;
	odom.pose.covariance[21] = FLT_MAX;
	odom.pose.covariance[28] =FLT_MAX;
	odom.pose.covariance[35] = 50;

	odom.twist.covariance[0] = .1; 
	odom.twist.covariance[7] = .1; 
	odom.twist.covariance[14] = 1000000000;
	odom.twist.covariance[21] = 1000000000;
	odom.twist.covariance[28] = 1000000000;
	odom.twist.covariance[35] = .1; 

    odom_pub.publish(odom);

    g_last_loop_time = current_time;
    r.sleep();
  }
}
