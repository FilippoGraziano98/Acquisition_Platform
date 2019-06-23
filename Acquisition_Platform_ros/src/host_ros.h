#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include "packets.h"

//to see better small movements in rviz
#define SCALE_FACTOR 4

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
		
		inline void setIMUOdomFrameId(const std::string& imu_odom_frame_id_) {
			_imu_odom_frame_id = imu_odom_frame_id_;
		}
		inline void setIMUOdomTopic(const std::string& imu_odom_topic_){
			_imu_odom_topic = imu_odom_topic_;
		}
		
		inline void setKFOdomFrameId(const std::string& kf_odom_frame_id_) {
			_kf_odom_frame_id = kf_odom_frame_id_;
		}
		inline void setKFOdomTopic(const std::string& kf_odom_topic_){
			_kf_odom_topic = kf_odom_topic_;
		}
		
		void odom_publish(OdometryPacket* odom_data);
		void imu_odom_publish(IMUOdometryPacket* imu_odom_data);
		void kf_odom_publish(KFOdometryPacket* kf_odom_data);
		void advertise();
	
	protected:
		std::string _odom_frame_id;
		std::string _odom_topic;
		
		std::string _imu_odom_frame_id;
		std::string _imu_odom_topic;
		
		std::string _kf_odom_frame_id;
		std::string _kf_odom_topic;
		
		ros::NodeHandle& _nh;
		ros::Publisher _odom_pub;
		ros::Publisher _imu_odom_pub;
		ros::Publisher _kf_odom_pub;
};
