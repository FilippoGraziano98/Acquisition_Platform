#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "../avr_common/uart.h" // this includes the printf and initializes it
#include "../avr_common/i2c.h"
#include "../avr_common/i2c_communication.h"
#include "imu_i2c.h"
#include "imu.h"

int main(void) {
  printf_init();
	printf("start!\n");
  I2C_Init();

	IMU_Init();
	printf("IMU initialized\n");
	
	unsigned char data = I2C_ReadRegister(0x68, 0x0);
	
	printf("at 0x0 found: %d\n",data);
	
	
	
	//MagnetometerSensor mg_data;
	//AccellerometerSensor ac_data;
	GyroscopeData gyro_data;
	//TermometerSensor term_data;
	while(1) {
/*		mg_data = IMU_ReadMagnetometer();	*/
/*		printf("===== MAGNETOMETER =====\n");*/
/*		printf(" - magnetometer: x-axis : %d\n", mg_data.magnet_x);*/
/*		printf(" - magnetometer: y-axis : %d\n", mg_data.magnet_y);*/
/*		printf(" - magnetometer: z-axis : %d\n", mg_data.magnet_z);*/
/*		printf("\n");*/
/*		*/
/*		ac_data = IMU_ReadAccellerometer();	*/
/*		printf("===== ACCELLEROMETER =====\n");*/
/*		printf(" - accellerometer: x-axis : %d\n", ac_data.accel_x);*/
/*		printf(" - accellerometer: y-axis : %d\n", ac_data.accel_y);*/
/*		printf(" - accellerometer: z-axis : %d\n", ac_data.accel_z);*/
/*		printf("\n");*/
		
		gyro_data = IMU_GyroscopeData();	
		printf("===== GYROSCOPE =====\n");
		printf(" - gyroscope: x-axis : %lf\n", gyro_data.gyro_x);
		printf(" - gyroscope: y-axis : %lf\n", gyro_data.gyro_y);
		printf(" - gyroscope: z-axis : %lf\n", gyro_data.gyro_z);
		printf("\n");
		
/*		term_data = IMU_ReadTermometer();	*/
/*		printf("===== TERMOMETER =====\n");*/
/*		printf(" - temperature : %d\n", term_data.temperature);*/
/*		printf("\n");*/
	}
}
