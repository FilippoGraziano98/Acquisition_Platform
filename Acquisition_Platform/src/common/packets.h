#pragma once
#include "packet_header.h"

#include "../firmware/firmware_constants.h"	//for NUM_ENCODERS, for IMU_N

//TODO set seq to a global epoque id
  // simple macro to initialize a packet
#define INIT_PACKET(var, id)			\
  {																\
    var.header.type=id;						\
    var.header.size=sizeof(var);	\
    var.header.seq=0;							\
  }																\

#define PACKET_MAX_ID 16	//TODO check

#define MIN_SIZE(t1,t2) ((t1<t2) ? t1 : t2)
#define MAX_SIZE(t1,t2) ((t1>t2) ? t1 : t2)

#define PACKET_MIN_SIZE (MIN_SIZE(sizeof(EchoPacket),\
													MAX_SIZE(sizeof(SystemStatusPacket),\
													MIN_SIZE(sizeof(EncoderPacket),\
													MAX_SIZE(sizeof(OdometryPacket),\
													MIN_SIZE(sizeof(IMUConfigurationPacket),\
													MIN_SIZE(sizeof(AccelerometerPacket),\
													MIN_SIZE(sizeof(GyroscopePacket),\
													MIN_SIZE(sizeof(MagnetometerPacket),sizeof(IMUOdometryPacket))))))))))
#define PACKET_MAX_SIZE (MAX_SIZE(sizeof(EchoPacket),\
													MAX_SIZE(sizeof(SystemStatusPacket),\
													MAX_SIZE(sizeof(EncoderPacket),\
													MAX_SIZE(sizeof(OdometryPacket),\
													MAX_SIZE(sizeof(IMUConfigurationPacket),\
													MAX_SIZE(sizeof(AccelerometerPacket),\
													MAX_SIZE(sizeof(GyroscopePacket),\
													MAX_SIZE(sizeof(MagnetometerPacket),sizeof(IMUOdometryPacket))))))))))

#pragma pack(push, 1)

//! sent from the pc to the robot causes
//! the robot to send it again to the pc
typedef struct {
  PacketHeader header;
	uint8_t info; //random info
} EchoPacket;
#define ECHO_PACKET_ID 0

typedef struct {
  PacketHeader header;
  uint32_t global_secs_count;	// secs since startup
	uint32_t idle_cycles; // count how long we spend doing nofin
	uint32_t rx_count;		// counts how many packets have been received
	uint32_t tx_count;		// counts how many packets have been transmitted
} SystemStatusPacket;
#define SYSTEM_STATUS_PACKET_ID 1

typedef struct {
  PacketHeader header;
	int32_t counters[NUM_ENCODERS];
} EncoderPacket;
#define ENCODER_PACKET_ID 2

typedef struct {
  PacketHeader header;
	#ifdef DEBUG_ODOM
	int32_t enc_left;
	int32_t enc_right;
	
	float delta_l;
	float delta_r;
	
	float delta_x;
	float delta_y;
	float delta_theta;
	#endif
	
	float odom_x;
	float odom_y;
	float odom_theta;	//(radianti)
	
	float translational_velocity;
	float rotational_velocity;
} OdometryPacket;
#define ODOMETRY_PACKET_ID 3

typedef struct {
  PacketHeader header;
	int16_t gyro_x_bias;		//gyroscope
	int16_t gyro_y_bias;
	int16_t gyro_z_bias;
	
	int16_t accel_x_bias;		//accelerometer
	int16_t accel_y_bias;
	int16_t accel_z_bias;
} IMUConfigurationPacket;
#define IMU_CONFIG_PACKET_ID 4


//NOTE: according to
	// https://learn.sparkfun.com/tutorials/data-types-in-arduino/all
	// float is 32bit-long on Arduino (as weel as on C)

//! ACCELEROMETER_PACKET_ID and GYROSCOPE_PACKET_ID
//! sent from the pc to the robot causes with just the header filled
//! the robot sends it again to the pc filled with the correct info
typedef struct {
  PacketHeader header;
	float accel_x;	//data measured in G-Forces
	float accel_y;		// [ note: G-forces -> a single G-force for us here on planet Earth is equivalent to 9.8 m/s^2 ]
	float accel_z;
} AccelerometerPacket;
#define ACCELEROMETER_PACKET_ID 5

typedef struct {
  PacketHeader header;
	float gyro_x;		//measure angular velocity (speed of rotation)
	float gyro_y;		//data measured in DPS
	float gyro_z;			// [ note: DPS, Degrees Per Second ]
} GyroscopePacket;
#define GYROSCOPE_PACKET_ID 6

typedef struct {
  PacketHeader header;
	float magnet_x;		//data measured in μT
	float magnet_y;
	float magnet_z;
} MagnetometerPacket;
#define MAGNETOMETER_PACKET_ID 7

typedef struct {
  PacketHeader header;
	
	float imu_odom_x;
	float imu_odom_y;
	float imu_odom_z;
	
	float translational_velocity_x_axis;
	float translational_velocity_y_axis;
	float translational_velocity_z_axis;
	
	float translational_acceleration_x_axis;
	float translational_acceleration_y_axis;
	float translational_acceleration_z_axis;
	
	float imu_yaw;	//rotation on z-axis (rad)
	float imu_pitch;//rotation on y-axis (rad)
	float imu_roll;	//rotation on x-axis (rad)
	
	float rotational_velocity_z_axis;
	float rotational_velocity_y_axis;
	float rotational_velocity_x_axis;
} IMUOdometryPacket;
#define IMU_ODOMETRY_PACKET_ID 8

#pragma pack(pop)
