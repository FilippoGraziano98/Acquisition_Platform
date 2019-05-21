#include "host_ros.h"

Host_ros::Host_ros(ros::NodeHandle& nh_) : _nh(nh_){
	_odom_frame_id = "";
	_odom_topic = "";
}

Host_ros::~Host_ros(){
}

void Host_ros::publish(OdometryPacket* odom_data) {
	nav_msgs::Odometry odom_msg = {};
		
	odom_msg.header.seq = odom_data->header.seq;
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.header.frame_id = _odom_frame_id;
	
	odom_msg.pose.pose.position.x = odom_data->odom_x*10;
	odom_msg.pose.pose.position.y = odom_data->odom_y*10;
	odom_msg.pose.pose.position.z = 0;
	odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_data->odom_theta);
	
	_odom_pub.publish(odom_msg);
	
}




void Host_ros::advertise(){
	_odom_pub = _nh.advertise<nav_msgs::Odometry>(_odom_topic, 10);
}
