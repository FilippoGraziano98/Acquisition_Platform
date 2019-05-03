#include "../../common/packet_header.h"
#include "../../common/packets.h"
#include "uart_packets.h"
#include "../imu/imu.h"
#include "../encoder/encoder.h"

#include "packet_handler.h"

// private packetHandler variable;
static PacketHandler packetHandler;

static uint8_t EchoPacketHandler(PacketHeader* pkt) {
	//all we must do with an echo packet is sending it back to host
	uint8_t size = UART_send_packet(pkt);
	
	if( size == pkt->size)
		return PACKET_OP_SUCCESS;
	else
		return PACKET_SEND_INCOMPLETE;
}

static uint8_t EncoderPacketHandler(PacketHeader* pkt) {
	if(pkt->type != ENCODER_PACKET_ID)
		return PACKET_OPS_VECTOR_CORRUPTED;
	
	//TODO do we need to update the seq/epoque ??
	//pkt->seq = ??
	
	EncoderPacket* enc_pkt = (EncoderPacket*)pkt;
	
	Encoders_getCounts(enc_pkt->counters);
		
	uint8_t size = UART_send_packet((PacketHeader*)enc_pkt);
	
	if( size == pkt->size)
		return PACKET_OP_SUCCESS;
	else
		return PACKET_SEND_INCOMPLETE;
}

static uint8_t IMUConfigPacketHandler(PacketHeader* pkt) {
	//we must fill in the gyroscope data
		//we also set the seq/epoque
	if(pkt->type != IMU_CONFIG_PACKET_ID)
		return PACKET_OPS_VECTOR_CORRUPTED;
	
	//TODO do we need to update the seq/epoque ??
	//pkt->seq = AcquisitionPlatform_getGlobalSeq();
	
	IMUConfigurationPacket* imu_config_pkt = (IMUConfigurationPacket*)pkt;
	
	IMU_getCalibrationData(&(imu_config_pkt->gyro_x_bias), &(imu_config_pkt->gyro_y_bias), &(imu_config_pkt->gyro_z_bias));
		
	uint8_t size = UART_send_packet((PacketHeader*)imu_config_pkt);
	
	if( size == pkt->size)
		return PACKET_OP_SUCCESS;
	else
		return PACKET_SEND_INCOMPLETE;
}

static uint8_t AccelerometerPacketHandler(PacketHeader* pkt) {
	//we must fill in the gyroscope data
		//we also set the seq/epoque
	if(pkt->type != ACCELEROMETER_PACKET_ID)
		return PACKET_OPS_VECTOR_CORRUPTED;
		
	AccelerometerPacket* accel_pkt = (AccelerometerPacket*)pkt;
	
	pkt->seq = IMU_getAccelerometer(&(accel_pkt->accel_x), &(accel_pkt->accel_y), &(accel_pkt->accel_z));
		
	uint8_t size = UART_send_packet((PacketHeader*)accel_pkt);
	
	if( size == pkt->size)
		return PACKET_OP_SUCCESS;
	else
		return PACKET_SEND_INCOMPLETE;
}

static uint8_t GyroscopePacketHandler(PacketHeader* pkt) {
	//we must fill in the gyroscope data
		//we also set the seq/epoque
	if(pkt->type != GYROSCOPE_PACKET_ID)
		return PACKET_OPS_VECTOR_CORRUPTED;
	
	GyroscopePacket* gyro_pkt = (GyroscopePacket*)pkt;
	
	pkt->seq = IMU_getGyroscope(&(gyro_pkt->gyro_x), &(gyro_pkt->gyro_y), &(gyro_pkt->gyro_z));
	
	uint8_t size = UART_send_packet((PacketHeader*)gyro_pkt);
	
	if( size == pkt->size)
		return PACKET_OP_SUCCESS;
	else
		return PACKET_SEND_INCOMPLETE;
}

static uint8_t MagnetometerPacketHandler(PacketHeader* pkt) {
	//we must fill in the gyroscope data
		//we also set the seq/epoque
	if(pkt->type != MAGNETOMETER_PACKET_ID)
		return PACKET_OPS_VECTOR_CORRUPTED;
	
	MagnetometerPacket* magnet_pkt = (MagnetometerPacket*)pkt;
	
	pkt->seq = IMU_getMagnetometer(&(magnet_pkt->magnet_x), &(magnet_pkt->magnet_y), &(magnet_pkt->magnet_z));
	
	uint8_t size = UART_send_packet((PacketHeader*)magnet_pkt);
	
	if( size == pkt->size)
		return PACKET_OP_SUCCESS;
	else
		return PACKET_SEND_INCOMPLETE;
}

void PacketHandler_init(void) {	
	//initialization of packetOps_vector
	packetHandler.packetOps_vector[ECHO_PACKET_ID] = EchoPacketHandler;
	
	packetHandler.packetOps_vector[ENCODER_PACKET_ID] = EncoderPacketHandler;
	
	packetHandler.packetOps_vector[IMU_CONFIG_PACKET_ID] = IMUConfigPacketHandler;
	
	packetHandler.packetOps_vector[ACCELEROMETER_PACKET_ID] = AccelerometerPacketHandler;
	packetHandler.packetOps_vector[GYROSCOPE_PACKET_ID] = GyroscopePacketHandler;
	packetHandler.packetOps_vector[MAGNETOMETER_PACKET_ID] = MagnetometerPacketHandler;
}

uint8_t PacketHandler_process(PacketHeader* pkt) {
	PacketType pkt_id =	pkt->type;
	if( pkt_id < 0 || pkt_id > PACKET_MAX_ID )
		return PACKET_ID_OUT_OF_RANGE;

	PacketOpFunctionType pkt_op = packetHandler.packetOps_vector[pkt_id];
	if( !pkt_op )
		return PACKET_OP_NOT_IMPLEMENTED;
	
	return (*pkt_op)(pkt);
}
