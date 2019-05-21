#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include "packets.h"

class Host_ros{
	public:
		Host_ros(ros::NodeHandle& nh_);
		~Host_ros();
		
		inline void setOdomFrameId(const std::string& odom_frame_id_) {
			_odom_frame_id = odom_frame_id_;
		}
		inline void setOdomTopic(const std::string& odom_topic_){
			_odom_topic = odom_topic_;
		}
		
		void publish(OdometryPacket* odom_pkt);
		void advertise();
	
	protected:
		std::string _odom_frame_id;
		std::string _odom_topic;
		
		ros::NodeHandle& _nh;
		ros::Publisher _odom_pub;
};
