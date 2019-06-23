#include "host_ros.h"

Host_ros::Host_ros(ros::NodeHandle& nh_) : _nh(nh_){
	_odom_frame_id = "";
	_odom_topic = "";
	_imu_odom_frame_id = "";
	_imu_odom_topic = "";
	_kf_odom_frame_id = "";
	_kf_odom_topic = "";
}

Host_ros::~Host_ros(){
}

void Host_ros::odom_publish(OdometryPacket* odom_data) {
	nav_msgs::Odometry odom_msg = {};
		
	odom_msg.header.seq = odom_data->header.seq;
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.header.frame_id = _odom_frame_id;
	
	odom_msg.pose.pose.position.x = odom_data->odom_x*SCALE_FACTOR;
	odom_msg.pose.pose.position.y = odom_data->odom_y*SCALE_FACTOR;
	odom_msg.pose.pose.position.z = 0;
	odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_data->odom_theta);
	
	_odom_pub.publish(odom_msg);
	
}

#define M_PI 3.141592654
#define M_180 180
void Host_ros::imu_odom_publish(IMUOdometryPacket* imu_odom_data) {
	nav_msgs::Odometry imu_odom_msg = {};
		
	imu_odom_msg.header.seq = imu_odom_data->header.seq;
	imu_odom_msg.header.stamp = ros::Time::now();
	imu_odom_msg.header.frame_id = _imu_odom_frame_id;
	
	imu_odom_msg.pose.pose.position.x = imu_odom_data->imu_odom_x*SCALE_FACTOR;
	imu_odom_msg.pose.pose.position.y = imu_odom_data->imu_odom_y*SCALE_FACTOR;
	imu_odom_msg.pose.pose.position.z = imu_odom_data->imu_odom_z*SCALE_FACTOR;
	
	float yaw_rad = imu_odom_data->imu_yaw;
	imu_odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rad);
	//TODO roll, pitch
	
	_imu_odom_pub.publish(imu_odom_msg);
}


void Host_ros::kf_odom_publish(KFOdometryPacket* kf_odom_data) {
	nav_msgs::Odometry kf_odom_msg = {};
		
	kf_odom_msg.header.seq = kf_odom_data->header.seq;
	kf_odom_msg.header.stamp = ros::Time::now();
	kf_odom_msg.header.frame_id = _kf_odom_frame_id;
	
	kf_odom_msg.pose.pose.position.x = kf_odom_data->kf_odom_x*SCALE_FACTOR;
	kf_odom_msg.pose.pose.position.y = kf_odom_data->kf_odom_y*SCALE_FACTOR;
	kf_odom_msg.pose.pose.position.z = 0;
	
	kf_odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(kf_odom_data->kf_odom_theta);
	
	_kf_odom_pub.publish(kf_odom_msg);
}


void Host_ros::advertise(){
	_odom_pub = _nh.advertise<nav_msgs::Odometry>(_odom_topic, 10);
	_imu_odom_pub = _nh.advertise<nav_msgs::Odometry>(_imu_odom_topic, 10);
	_kf_odom_pub = _nh.advertise<nav_msgs::Odometry>(_kf_odom_topic, 10);
}
