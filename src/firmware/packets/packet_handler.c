#include "packet_handler.h"

#include "acquisition_platform.h"
#include "../../common/packet_header.h"
#include "../../common/packets.h"
#include "uart_packets.h"

#include "../imu/imu.h"

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

static uint8_t IMUConfigPacketHandler(PacketHeader* pkt) {
	//we must fill in the gyroscope data
		//we also set the seq/epoque
	if(pkt->type != IMU_CONFIG_PACKET_ID)
		return PACKET_OPS_VECTOR_CORRUPTED;
		
	//we update the seq/epoque
	pkt->seq = AcquisitionPlatform_getGlobalSeq();
	
	IMUConfigurationPacket* imu_config_pkt = (IMUConfigurationPacket*)pkt;
	
	GyroscopeCalibrationBiases gyro_biases = AcquisitionPlatform_getGyroscopeBiases();
		
	imu_config_pkt->gyro_x_bias = gyro_biases.gyro_x_bias;
	imu_config_pkt->gyro_y_bias = gyro_biases.gyro_y_bias;
	imu_config_pkt->gyro_z_bias = gyro_biases.gyro_z_bias;
		
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
	
	uint16_t global_seq = AcquisitionPlatform_getGlobalSeq();
	
	//if host already has updated info, no need to send them again
	//if( pkt->seq == global_seq )
		//return PACKET_OP_SUCCESS;
	
	//else we update the seq/epoque
	pkt->seq = global_seq;
	
	AccelerometerPacket* accel_pkt = (AccelerometerPacket*)pkt;
	
	AccelerometerData acl_data = AcquisitionPlatform_getAccelerometer();
		
	accel_pkt->accel_x = acl_data.accel_x;
	accel_pkt->accel_y = acl_data.accel_y;
	accel_pkt->accel_z = acl_data.accel_z;
		
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
	
	uint16_t global_seq = AcquisitionPlatform_getGlobalSeq();
	
	//if host already has updated info, no need to send them again
	//if( pkt->seq == global_seq )
		//return PACKET_OP_SUCCESS;
	
	//else we update the seq/epoque
	pkt->seq = global_seq;
	
	GyroscopePacket* gyro_pkt = (GyroscopePacket*)pkt;
	
	GyroscopeData gyro_data = AcquisitionPlatform_getGyroscope();
	
	gyro_pkt->gyro_x = gyro_data.gyro_x;
	gyro_pkt->gyro_y = gyro_data.gyro_y;
	gyro_pkt->gyro_z = gyro_data.gyro_z;
	
	#ifdef DEBUG_RAW
	gyro_pkt->raw_x = packetHandler.acq_pl->gyroscope_data.raw_x;
	gyro_pkt->raw_y = packetHandler.acq_pl->gyroscope_data.raw_y;
	gyro_pkt->raw_z = packetHandler.acq_pl->gyroscope_data.raw_z;
	#endif
	
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
	
	uint16_t global_seq = AcquisitionPlatform_getGlobalSeq();
	
	//if host already has updated info, no need to send them again
	//if( pkt->seq == global_seq )
		//return PACKET_OP_SUCCESS;
	
	//else we update the seq/epoque
	pkt->seq = global_seq;
	
	MagnetometerPacket* magnet_pkt = (MagnetometerPacket*)pkt;
	
	MagnetometerData mgn_data = AcquisitionPlatform_getMagnetometer();
	
	magnet_pkt->magnet_x = mgn_data.magnet_x;
	magnet_pkt->magnet_y = mgn_data.magnet_y;
	magnet_pkt->magnet_z = mgn_data.magnet_z;
	
	uint8_t size = UART_send_packet((PacketHeader*)magnet_pkt);
	
	if( size == pkt->size)
		return PACKET_OP_SUCCESS;
	else
		return PACKET_SEND_INCOMPLETE;
}

void PacketHandler_init(void) {	
	//initialization of packetOps_vector
	packetHandler.packetOps_vector[ECHO_PACKET_ID] = EchoPacketHandler;
	
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
