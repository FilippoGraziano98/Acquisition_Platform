#include <avr/io.h>
#include <util/delay.h>

#include "../avr_common/i2c_communication.h"
#include "imu_i2c.h"

void IMU_Init(void){
	//resets the internal registers and restores the default settings.
	I2C_WriteBits(IMU_DEVICE, PWR_MGMT_1, 0x80, 0x80);
	_delay_ms(500);
	
	//sets DLPF to have gyroscope bandwidth to 5Hz
	I2C_WriteRegister(IMU_DEVICE, CONFIG, 0x06);
	_delay_ms(10);
	
	//sets Gyro Full Scale RANGE to 250dps (degrees/sec)
		//consequently we have a low range, but a high sensitivity!!
	I2C_WriteBits(IMU_DEVICE, GYRO_CONFIG, (1<<4)|(1<<3), 0x00);
	_delay_ms(10);
	
	//sets Accel Full Scale to 2g
	I2C_WriteBits(IMU_DEVICE, ACCEL_CONFIG, (1<<4)|(1<<3), 0x00);
	_delay_ms(10);
	//sets Accel bandwidth to 5Hz
	I2C_WriteRegister(IMU_DEVICE, ACCEL_CONFIG2, 0x06);
	_delay_ms(10);
	
	//sets PowerManagement Registers
		//auto selects the best available clock source
	I2C_WriteBits(IMU_DEVICE, PWR_MGMT_1, (1<<2)|(1<<1)|1, 0x01);
		//disables sleep mode
	I2C_WriteBits(IMU_DEVICE, PWR_MGMT_1, 1<<6, 1<<6);
	_delay_ms(10);
		//sets all sensors to on
	I2C_WriteBits(IMU_DEVICE, PWR_MGMT_2, (1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|1, 0x00);
	_delay_ms(10);
	
	//i2c_master interface pins(SCL, SDA)
		//will go into ‘bypass mode’
		//when the i2c master interface is disabled
	I2C_WriteRegister(IMU_DEVICE, INT_PIN_CFG, 0x02);
	_delay_ms(10);
	
	//setting for Magnetometer 16bit output
		//and Single measurement mode [0x11]
		//TODO change to 0x12 to witch to Continuous measurement mode 1
	I2C_WriteRegister(IMU_DEVICE, CNTL1, 0x11);
	_delay_ms(10);
}

MagnetometerSensor IMU_ReadMagnetometer(void) {
	//I will read 7 regs starting from HXL because I'll also read ST2
		//data read must end reading ST2 (from datasheet)
	unsigned char data[7];
	I2C_ReadNRegisters(IMU_DEVICE, HXL, 7, data);
	
	MagnetometerSensor mg_data;
	mg_data.magnet_x = ( ((int16_t) data[1]) << 8 ) + data[0];
	mg_data.magnet_y = ( ((int16_t) data[3]) << 8 ) + data[2];
	mg_data.magnet_z = ( ((int16_t) data[5]) << 8 ) + data[4];
	
	return mg_data;
}

AccellerometerSensor IMU_ReadAccellerometer(void) {
	unsigned char data[6];
	I2C_ReadNRegisters(IMU_DEVICE, ACCEL_XOUT_H, 6, data);
	
	AccellerometerSensor ac_data;
	ac_data.accel_x = ( ((int16_t) data[0]) << 8 ) + data[1];
	ac_data.accel_y = ( ((int16_t) data[2]) << 8 ) + data[3];
	ac_data.accel_z = ( ((int16_t) data[4]) << 8 ) + data[5];
	
	return ac_data;
}

GyroscopeSensor IMU_ReadGyroscope(void) {
	unsigned char data[6];
	I2C_ReadNRegisters(IMU_DEVICE, GYRO_XOUT_H, 6, data);
	
	GyroscopeSensor gyro_data;
	gyro_data.gyro_x = ( ((int16_t) data[0]) << 8 ) + data[1];
	gyro_data.gyro_y = ( ((int16_t) data[2]) << 8 ) + data[3];
	gyro_data.gyro_z = ( ((int16_t) data[4]) << 8 ) + data[5];
	
	return gyro_data;
}

TermometerSensor IMU_ReadTermometer(void) {
	TermometerSensor term_data;
	term_data.temperature = (I2C_ReadRegister(IMU_DEVICE, TEMP_OUT_H) << 8) + I2C_ReadRegister(IMU_DEVICE, TEMP_OUT_L);
	
	return term_data;
}

/*void IMU_ReadSensors(void){*/
/*	//magnetometer*/
/*	mag_x=(twi_read_data(mag_9250_ID,HXH)<<8)+twi_read_data(mag_9250_ID,HXL);*/
/*	mag_y=(twi_read_data(mag_9250_ID,HYH)<<8)+twi_read_data(mag_9250_ID,HYL);*/
/*	mag_z=(twi_read_data(mag_9250_ID,HZH)<<8)+twi_read_data(mag_9250_ID,HZL);*/
/*	//checks for sensor overflow??*/
/*	twi_read_data(mag_9250_ID,ST2);*/
/*	*/
/*	//accellerometer*/
/*	x_in=(twi_read_data(mpu_9250_ID,ACCEL_XOUT_H)<<8)+twi_read_data(mpu_9250_ID,ACCEL_XOUT_L)>>5;*/
/*	y_in=(twi_read_data(mpu_9250_ID,ACCEL_YOUT_H)<<8)+twi_read_	data(mpu_9250_ID,ACCEL_YOUT_L)>>5;*/
/*	*/
/*	//gyroscope*/
/*	roll_in=((twi_read_data(mpu_9250_ID,GYRO_XOUT_H)<<8)+twi_read_data(mpu_9250_ID,GYRO_XOUT_L))>>5;*/
/*	pitch_in=((twi_read_data(mpu_9250_ID,GYRO_YOUT_H)<<8)+twi_read_data(mpu_9250_ID,GYRO_YOUT_L))>>5;*/
/*	yaw_in=((twi_read_data(mpu_9250_ID,GYRO_ZOUT_H)<<8)+twi_read_data(mpu_9250_ID,GYRO_ZOUT_L))>>5;*/
/*	//temperature*/
/*	temp_in=(twi_read_data(mpu_9250_ID,TEMP_OUT_H)<<8)+twi_read_data(mpu_9250_ID,TEMP_OUT_L);*/
/*}*/

