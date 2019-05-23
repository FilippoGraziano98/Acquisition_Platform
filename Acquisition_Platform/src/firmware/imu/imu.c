#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include <string.h>
#ifdef DEBUG_PRINTF_
#include <stdio.h>
#endif

#include "../avr_common/i2c_communication.h"

#include "imu.h"
#include "imu_odometry.h"

static IMU_t IMU;

static void IMU_StructInit(void) {
	memset(&IMU, 0, sizeof(IMU_t));
	INIT_PACKET(IMU.imu_config_values, IMU_CONFIG_PACKET_ID);
	INIT_PACKET(IMU.accel_values, ACCELEROMETER_PACKET_ID);
	INIT_PACKET(IMU.gyro_values, GYROSCOPE_PACKET_ID);
	INIT_PACKET(IMU.magnet_values, MAGNETOMETER_PACKET_ID);
}

static void IMU_ConfigRegs(void) {
	//resets the internal registers and restores the default settings.
	//I2C_WriteBits(ACCELGYRO_DEVICE, PWR_MGMT_1, 0x80, 0x80);
	I2C_WriteRegister(ACCELGYRO_DEVICE, PWR_MGMT_1, 0x80);
	_delay_ms(500);
		
	//sets Gyro Full Scale RANGE to 250dps (degrees/sec)
		//consequently we have a low range, but a high sensitivity!!
	//and 0-es Fchoice_b in order to be able to change gyoscope bandwidth
	//I2C_WriteBits(ACCELGYRO_DEVICE, GYRO_CONFIG, (1<<4)|(1<<3)|(1<<1)|1, 0x00);
	I2C_WriteRegister(ACCELGYRO_DEVICE, GYRO_CONFIG, 0x00);
	_delay_ms(10);
	
	//sets DLPF to have gyroscope bandwidth to 250Hz,
		// [Bandwidth is the highest frequency signal that can be sampled without aliasing
					// by the specified Output Data Rate]
		// Per the Nyquist sampling criterion, bandwidth is half the Output Data Rate.
		// so we will sample data at Output Data Rate = 500 Hz
	//I2C_WriteRegister(ACCELGYRO_DEVICE, CONFIG, 0x06);// 5Hz
	I2C_WriteRegister(ACCELGYRO_DEVICE, CONFIG, 0x00);
	_delay_ms(10);
	
	//sets Accel Full Scale to 2g
	//I2C_WriteBits(ACCELGYRO_DEVICE, ACCEL_CONFIG, (1<<4)|(1<<3), 0x00);
	I2C_WriteRegister(ACCELGYRO_DEVICE, ACCEL_CONFIG, 0x00);
	_delay_ms(10);
	//sets Accel bandwidth to 218.1 Hz
	//I2C_WriteRegister(ACCELGYRO_DEVICE, ACCEL_CONFIG2, 0x06);// 5Hz
	I2C_WriteRegister(ACCELGYRO_DEVICE, ACCEL_CONFIG2, 0x00);
	_delay_ms(10);

		//disables sleep mode
	//I2C_WriteBits(ACCELGYRO_DEVICE, PWR_MGMT_1, 1<<6, 0);
	//sets PowerManagement Registers
		//auto selects the best available clock source
	//I2C_WriteBits(ACCELGYRO_DEVICE, PWR_MGMT_1, (1<<2)|(1<<1)|1, 0x01);
	I2C_WriteRegister(ACCELGYRO_DEVICE, PWR_MGMT_1, 0x01);
	
	_delay_ms(10);
		//sets all sensors to on
	//I2C_WriteBits(ACCELGYRO_DEVICE, PWR_MGMT_2, (1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|1, 0x00);
	I2C_WriteRegister(ACCELGYRO_DEVICE, PWR_MGMT_2, 0x00);
	_delay_ms(10);
	
	//i2c_master interface pins(SCL, SDA)
		//will go into ‘bypass mode’
		//when the i2c master interface is disabled
	I2C_WriteRegister(ACCELGYRO_DEVICE, INT_PIN_CFG, 0x02);
	_delay_ms(10);
	
	//setting for Magnetometer 16bit output
		//and Continuous measurement mode [0x12]
	I2C_WriteRegister(MAGNET_DEVICE, CNTL1, 0x12);
	_delay_ms(10);
}

/* IMU_<Sensor>Raw
 * 	reads raw data from IMU, and stores it in global IMU_rawData
 *	NOTE: these functions are called by the timer ISR
 *			and they are defined later in this file
 */
static void IMU_AccelerometerRaw(void);//deprecated
static void IMU_GyroscopeRaw(void);//deprecated
static void IMU_AccelGyroRaw(void);
static void IMU_MagnetometerRaw(void);
static void IMU_TermometerRaw(void);

/*
 * 	[ https://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html ]
 * The AVR hardware clears the global interrupt flag in SREG before entering an interrupt vector.
 * 		Thus, normally interrupts will remain disabled inside the handler until the handler exits (reti).
 * To avoid this behaviour (re-enabling the global interrupt flag as early as possible),
 * 		in order to not defer any other interrupt more than absolutely needed
 *			you can use the isr_attribute: ISR_NOBLOCK
 *
 * Using ISR_NOBLOCK, the isr runs with global interrupts enabled.
 * 	The interrupt enable flag is activated by the compiler as early as possible within the ISR
 * 		to ensure minimal processing delay for nested interrupts (i.e. other interrupts).
 */
/*
 * In our case as communication with the sensor through I2C is very slow
 *		[ this ISR has a mean duration of 10 ms (empiric measure) ]
 *	and we don't want to delay this much the communication with the host (uart)
 */
ISR(TIMER3_COMPA_vect) {
	//IMU_AccelerometerRaw();
	//IMU_GyroscopeRaw();
	IMU_AccelGyroRaw();
	IMU_MagnetometerRaw();
	//IMU_TermometerRaw();

	IMU.imu_time_seq++;
}

static void IMU_setPeriodicDataUpdate(uint16_t frequency) {
  uint16_t period_ms = 1000 / frequency; //from a frequency in Hz, we get a period in millisecs
  
  // configure timer1, prescaler : 256, CTC (Clear Timer on Compare match)
  TCCR3A = 0;
  TCCR3B = (1 << WGM12) | (1 << CS12); 
  
  /*
	 * cpu frequency 16MHz = 16.000.000 Hz
	 * prescaler 256
	 *	-->> TCNT1 increased at a frequency of 16.000.000/256 Hz = 62500 Hz
	 *	so 1 ms will correspond do 62.5 counts
	 */
  OCR3A = (uint16_t)(62.5 * period_ms);

	// timer-interrupt enabling will be executed atomically (no other interrupts)
		// and ATOMIC_FORCEON ensures Global Interrupt Status flag bit in SREG set afetrwards
		// (sei() not needed)
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
  	TIMSK3 |= (1 << OCIE3A);  // enable the timer interrupt (istruz. elementare, no interrupt)
  }
}

void IMU_Init(void) {
	IMU_StructInit();
	IMU_ConfigRegs();
	IMU_setPeriodicDataUpdate(IMU_UPDATE_RATE);
}


/*
 * Accelerometer Sensor [ https://learn.sparkfun.com/tutorials/accelerometer-basics ]
 * 	ACCEL_XOUT = Accel_Sensitivity * X_acceleration
 *		where:
 *			-250dps ACCEL_XOUT : value read from the sensor (a change in the voltage)
 *			- X_acceleration : variation of acceleration / force
 *			- Accel_Sensitivity : indicates how much the voltage changes for a given acceleration
 *					we set our gyroscope to have Full Scale Range equal to (+/-)2 G
 *							[ note: G-forces -> a single G-force for us here on planet Earth is equivalent to 9.8 m/s^2 ]
 *						and we read a 16-bits value in 2's complement (+/-)2^15 mV
 *					==>> sensitivity = 2^15 / 2
 */
static void IMU_Accelerometer_Callback(uint8_t* buffer, uint8_t buflen) {
	if(buflen != 6)	//this is the callback of a read of 6 bytes
		return;
	
	IMU.accel_raw_x = ( ((int16_t) buffer[0]) << 8 ) + buffer[1];
	IMU.accel_raw_y = ( ((int16_t) buffer[2]) << 8 ) + buffer[3];
	IMU.accel_raw_z = ( ((int16_t) buffer[4]) << 8 ) + buffer[5];

	IMU.accel_raw_flag = VALID;
	
	#ifdef DEBUG_PRINTF_
	printf("IMU Accel Callback, raw x: %d, raw y: %d, raw z: %d\n", IMU.accel_raw_x, IMU.accel_raw_y, IMU.accel_raw_z);
	#endif

	float accel_sensitivity = (uint16_t)(1<<15) / (float)2;
		
	IMU.accel_values.accel_x = (float)(IMU.accel_raw_x - IMU.imu_config_values.accel_x_bias) / accel_sensitivity;
	IMU.accel_values.accel_y = (float)(IMU.accel_raw_y - IMU.imu_config_values.accel_y_bias) / accel_sensitivity;
	IMU.accel_values.accel_z = (float)(IMU.accel_raw_z - IMU.imu_config_values.accel_z_bias) / accel_sensitivity;

	IMU.accel_seq++;
}
static void IMU_AccelerometerRaw(void) {
	IMU.accel_raw_flag = INVALID;

	I2C_ReadNRegisters(ACCELGYRO_DEVICE, ACCEL_XOUT_H, 6, IMU_Accelerometer_Callback);
}


/*
 * Gyroscope Sensor [ https://learn.sparkfun.com/tutorials/gyroscope ]
 *	GYRO_XOUT = Gyro_Sensitivity * X_angular_rate
 *		where:
 *			- GYRO_XOUT : value read from the sensor (a change in the voltage)
 *			- X_angular_rate : variation of angular rotation
 *			- Gyro_Sensitivity : indicates how much the voltage changes for a given angular velocity (mV / DPS)
 *					we set our gyroscope to have Full Scale Range equal to (+/-)250 DPS
 *						and we read a 16-bits value in 2's complement (+/-)2^15 mV
 *					==>> sensitivity = 2^15 / 250
 */
static void IMU_Gyroscope_Callback(uint8_t* buffer, uint8_t buflen) {
	if(buflen != 6)	//this is the callback of a read of 6 bytes
		return;
	
	IMU.gyro_raw_x = ( ((int16_t) buffer[0]) << 8 ) + buffer[1];
	IMU.gyro_raw_y = ( ((int16_t) buffer[2]) << 8 ) + buffer[3];
	IMU.gyro_raw_z = ( ((int16_t) buffer[4]) << 8 ) + buffer[5];

	IMU.gyro_raw_flag = VALID;

	#ifdef DEBUG_PRINTF_
	printf("IMU Gyro Callback, raw x: %d, raw y: %d, raw z: %d\n", IMU.gyro_raw_x, IMU.gyro_raw_y, IMU.gyro_raw_z);
	#endif
		
	float gyro_sensitivity = (uint16_t)(1<<15) / (float)250;
	
	IMU.gyro_values.gyro_x = (float)(IMU.gyro_raw_x - IMU.imu_config_values.gyro_x_bias) / gyro_sensitivity;
	IMU.gyro_values.gyro_y = (float)(IMU.gyro_raw_y - IMU.imu_config_values.gyro_y_bias) / gyro_sensitivity;
	IMU.gyro_values.gyro_z = (float)(IMU.gyro_raw_z - IMU.imu_config_values.gyro_z_bias) / gyro_sensitivity;

	IMU.gyro_seq++;
}
static void IMU_GyroscopeRaw(void) {
	IMU.gyro_raw_flag = INVALID;

	I2C_ReadNRegisters(ACCELGYRO_DEVICE, GYRO_XOUT_H, 6, IMU_Gyroscope_Callback);
}


static void IMU_AccelGyro_Callback(uint8_t* buffer, uint8_t buflen) {
	if(buflen != 14)	//this is the callback of a read of 14 bytes
		return;
	
	IMU.accel_raw_x = ( ((int16_t) buffer[0]) << 8 ) + buffer[1];
	IMU.accel_raw_y = ( ((int16_t) buffer[2]) << 8 ) + buffer[3];
	IMU.accel_raw_z = ( ((int16_t) buffer[4]) << 8 ) + buffer[5];

	IMU.accel_raw_flag = VALID;

	//GYRO_XOUT_H is the 8-th byte in buffer
	uint8_t gyro_offest_buf = 8;
	IMU.gyro_raw_x = ( ((int16_t) buffer[gyro_offest_buf+0]) << 8 ) + buffer[gyro_offest_buf+1];
	IMU.gyro_raw_y = ( ((int16_t) buffer[gyro_offest_buf+2]) << 8 ) + buffer[gyro_offest_buf+3];
	IMU.gyro_raw_z = ( ((int16_t) buffer[gyro_offest_buf+4]) << 8 ) + buffer[gyro_offest_buf+5];

	IMU.gyro_raw_flag = VALID;

	#ifdef DEBUG_PRINTF_
	printf("IMU Accel Callback, raw x: %d, raw y: %d, raw z: %d\n", IMU.accel_raw_x, IMU.accel_raw_y, IMU.accel_raw_z);
	printf("IMU Gyro Callback, raw x: %d, raw y: %d, raw z: %d\n\n", IMU.gyro_raw_x, IMU.gyro_raw_y, IMU.gyro_raw_z);
	#endif
	
	//accelerometer
	float accel_sensitivity = (uint16_t)(1<<15) / (float)2;
		
	IMU.accel_values.accel_x = (float)(IMU.accel_raw_x - IMU.imu_config_values.accel_x_bias) / accel_sensitivity;
	IMU.accel_values.accel_y = (float)(IMU.accel_raw_y - IMU.imu_config_values.accel_y_bias) / accel_sensitivity;
	IMU.accel_values.accel_z = (float)(IMU.accel_raw_z - IMU.imu_config_values.accel_z_bias) / accel_sensitivity;

	IMU.accel_seq++;
	
	//gyroscope
	float gyro_sensitivity = (uint16_t)(1<<15) / (float)250;
	
	IMU.gyro_values.gyro_x = (float)(IMU.gyro_raw_x - IMU.imu_config_values.gyro_x_bias) / gyro_sensitivity;
	IMU.gyro_values.gyro_y = (float)(IMU.gyro_raw_y - IMU.imu_config_values.gyro_y_bias) / gyro_sensitivity;
	IMU.gyro_values.gyro_z = (float)(IMU.gyro_raw_z - IMU.imu_config_values.gyro_z_bias) / gyro_sensitivity;

	IMU.gyro_seq++;
	
	IMU_OdometryUpdate(&(IMU.accel_values), &(IMU.gyro_values));
}
static void IMU_AccelGyroRaw(void) {
	IMU.accel_raw_flag = INVALID;
	IMU.gyro_raw_flag = INVALID;
	
	//reading registers from ACCEL_XOUT_H to GYRO_ZOUT_L
	I2C_ReadNRegisters(ACCELGYRO_DEVICE, ACCEL_XOUT_H, 14, IMU_AccelGyro_Callback);
}

/*
 * Magnetometer Sensor [ from datasheet ]
 * 	X_magnetic_field = Magnet_Resolution * register_XOUT
 *		where:
 *			- register_XOUT : value read from the sensor (a change in the voltage)
 *			- X_magnetic_field : magnetic field measured on the x-axis
 *			- Magnet_Resolution : indicates how much the voltage changes for a given magnetic field
 *					Full scale measurement range is +/- 4912 μT [datasheet says 4800, register map says 4912]
 *						and we read a 16-bits value in 2's complement (+/-)2^15 mV
 *							( more precisely range is -32760 ~ 32760, and not +/- 32768 )
 *					[ NOTE: Magnet_Resolution = 1/Magnet_Sensitivity ]
 *					==>> Magnet_Resolution =~ 4912 / 32760 =~ 0.14993894993894993
 */
static void IMU_Magnetometer_Callback(uint8_t* buffer, uint8_t buflen) {
	if(buflen != 7)	//this is the callback of a read of 7 bytes
		return;
	
	IMU.magnet_raw_x = ( ((int16_t) buffer[1]) << 8 ) + buffer[0];
	IMU.magnet_raw_y = ( ((int16_t) buffer[3]) << 8 ) + buffer[2];
	IMU.magnet_raw_z = ( ((int16_t) buffer[5]) << 8 ) + buffer[4];

	IMU.magnet_raw_flag = VALID;

	#ifdef DEBUG_PRINTF_
	printf("IMU Magnet Callback, raw x: %d, raw y: %d, raw z: %d\n", IMU.magnet_raw_x, IMU.magnet_raw_y, IMU.magnet_raw_z);
	#endif
	
	float mg_resolution = (float)4912 / 32760;
		
	IMU.magnet_values.magnet_x = (float)(IMU.magnet_raw_x) * mg_resolution;
	IMU.magnet_values.magnet_y = (float)(IMU.magnet_raw_y) * mg_resolution;
	IMU.magnet_values.magnet_z = (float)(IMU.magnet_raw_z) * mg_resolution;

	IMU.magnet_seq++;
}
static void IMU_MagnetometerRaw(void) {
	IMU.magnet_raw_flag = INVALID;

	//I will read 7 regs starting from HXL because I'll also read ST2
		//data read must end reading ST2 (from datasheet)
	I2C_ReadNRegisters(MAGNET_DEVICE, HXL, 7, IMU_Magnetometer_Callback);
}


static void IMU_Termometer_Callback(uint8_t* buffer, uint8_t buflen) {
	if(buflen != 2)	//this is the callback of a read of 7 bytes
		return;
	
	IMU.temperature_raw = ( ((int16_t) buffer[0]) << 8 ) + buffer[1];

	IMU.temp_raw_flag = VALID;
}
static void IMU_TermometerRaw(void) {
	IMU.temp_raw_flag = INVALID;

	I2C_ReadNRegisters(ACCELGYRO_DEVICE, TEMP_OUT_H, 2, IMU_Termometer_Callback);
}


#define CALIBRATION_SAMPLES	64
#define CALIBRATION_SAMPLES_LOG	6
/*
As with any sensor, the values you measure will contain some amount of error or bias.
You can see gyro bias by measuring the output when the gyro is still.
These errors are sometimes called bias drift or bias instability.

The temperature of the sensor greatly affects the bias.
To help minimize the source of this error, most gyros have a built in temperature sensor.
Thus, you are able to read the temperature of the sensor and correct or any temperature dependent changes.

In order to correct for these errors, the gyro must be calibrated.

This is usually done by keeping the gyro still and zeroing all of the readings in your code.
*/
static void IMU_GyroscopeCalibration(void) {
	uint32_t gyro_x_sum=0, gyro_y_sum=0, gyro_z_sum=0;
	
	#ifdef DEBUG_PRINTF_
	printf("IMU_GyroscopeCalibration\n");
	#endif
	
	uint8_t i;
	for(i=0; i<CALIBRATION_SAMPLES; i++) {
		//waits for gyroscope data to be valid
		while(IMU.gyro_raw_flag == INVALID)
			_delay_ms(1);
		
		
		gyro_x_sum += IMU.gyro_raw_x;
		gyro_y_sum += IMU.gyro_raw_y;
		gyro_z_sum += IMU.gyro_raw_z;
		
		#ifdef DEBUG_PRINTF_
		printf("    %d) x: %d [sum: %ld], y: %d [sum: %ld], z: %d [sum: %ld]\n", i, IMU.gyro_raw_x, gyro_x_sum, IMU.gyro_raw_y, gyro_y_sum, IMU.gyro_raw_z, gyro_z_sum);
		#endif
	}
	
	IMU.imu_config_values.gyro_x_bias = gyro_x_sum >> CALIBRATION_SAMPLES_LOG;
	IMU.imu_config_values.gyro_y_bias = gyro_y_sum >> CALIBRATION_SAMPLES_LOG;
	IMU.imu_config_values.gyro_z_bias = gyro_z_sum >> CALIBRATION_SAMPLES_LOG;
	
	#ifdef DEBUG_PRINTF_
	printf("  x: %d, y: %d, z: %d\n", IMU.gyro_x_bias, IMU.gyro_y_bias, IMU.gyro_z_bias);
	#endif
}

static void IMU_AccelerometerCalibration(void) {
	uint32_t accel_x_sum=0, accel_y_sum=0, accel_z_sum=0;
	
	#ifdef DEBUG_PRINTF_
	printf("IMU_AccelerometerCalibration\n");
	#endif
	
	uint8_t i;
	for(i=0; i<CALIBRATION_SAMPLES; i++) {
		//waits for gyroscope data to be valid
		while(IMU.accel_raw_flag == INVALID)
			_delay_ms(1);
		
		
		accel_x_sum += IMU.accel_raw_x;
		accel_y_sum += IMU.accel_raw_y;
		accel_z_sum += IMU.accel_raw_z;
		
		#ifdef DEBUG_PRINTF_
		printf("    %d) x: %d [sum: %ld], y: %d [sum: %ld], z: %d [sum: %ld]\n", i, IMU.accel_raw_x, accel_x_sum, IMU.accel_raw_y, accel_y_sum, IMU.accel_raw_z, accel_z_sum);
		#endif
	}
	
	IMU.imu_config_values.accel_x_bias = accel_x_sum >> CALIBRATION_SAMPLES_LOG;
	IMU.imu_config_values.accel_y_bias = accel_y_sum >> CALIBRATION_SAMPLES_LOG;
	IMU.imu_config_values.accel_z_bias = accel_z_sum >> CALIBRATION_SAMPLES_LOG;
	
	#ifdef DEBUG_PRINTF_
	printf("  x: %d, y: %d, z: %d\n", IMU.accel_x_bias, IMU.accel_y_bias, IMU.accel_z_bias);
	#endif
}

void IMU_Calibration(void) {
	IMU_GyroscopeCalibration();
	IMU_AccelerometerCalibration();
}

void IMU_getCalibrationData(IMUConfigurationPacket* config_pkt) {
	memcpy((void*)config_pkt+sizeof(PacketHeader), (void*)&(IMU.imu_config_values)+sizeof(PacketHeader), sizeof(IMUConfigurationPacket)-sizeof(PacketHeader));
}

uint16_t IMU_getAccelerometer(AccelerometerPacket* accel_pkt) {
	memcpy((void*)accel_pkt+sizeof(PacketHeader), (void*)&(IMU.accel_values)+sizeof(PacketHeader), sizeof(AccelerometerPacket)-sizeof(PacketHeader));
	return IMU.accel_seq;
}
uint16_t IMU_getGyroscope(GyroscopePacket* gyro_pkt) {
	memcpy((void*)gyro_pkt+sizeof(PacketHeader), (void*)&(IMU.gyro_values)+sizeof(PacketHeader), sizeof(GyroscopePacket)-sizeof(PacketHeader));
	return IMU.gyro_seq;
}
uint16_t IMU_getMagnetometer(MagnetometerPacket* magnet_pkt) {
	memcpy((void*)magnet_pkt+sizeof(PacketHeader), (void*)&(IMU.magnet_values)+sizeof(PacketHeader), sizeof(MagnetometerPacket)-sizeof(PacketHeader));
	return IMU.magnet_seq;
}


uint8_t IMU_sendIMUDataToHost(void) {
	uint8_t res, ret = 0;
	
	IMU.accel_values.header.seq = IMU.accel_seq;
	res = UART_send_packet((PacketHeader*)&(IMU.accel_values));
	if (res > 0)
		ret++;
	
	IMU.gyro_values.header.seq = IMU.gyro_seq;
	res = UART_send_packet((PacketHeader*)&(IMU.gyro_values));
	if (res > 0)
		ret++;
	
	IMU.magnet_values.header.seq = IMU.magnet_seq;
	res = UART_send_packet((PacketHeader*)&(IMU.magnet_values));
	if (res > 0)
		ret++;
	
	return ret;
}
