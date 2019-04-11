#include <stdlib.h>

#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "acquisition_platform.h"

// private AcquisitionPlatform variable;
static AcquisitionPlatform acquisition_platform;

void AcquisitionPlatform_init(void) {	
	acquisition_platform.global_seq = 0;
	
	acquisition_platform.accelerometer_data = (AccelerometerData){ .accel_x = 0, .accel_y = 0, .accel_z = 0 };
	acquisition_platform.gyroscope_data = (GyroscopeData){ .gyro_x = 0, .gyro_y = 0, .gyro_z = 0 };
	acquisition_platform.magnetometer_data = (MagnetometerData){ .magnet_x = 0, .magnet_y = 0, .magnet_z = 0 };
}

void AcquisitionPlatform_imuCalibrate(void) {
	acquisition_platform.gyroscope_biases = IMU_GyroscopeCalibration();
}

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
ISR(TIMER1_COMPA_vect, ISR_NOBLOCK) {
//void AcquisitionPlatform_imuUpdate(void) {
	acquisition_platform.accelerometer_data = IMU_AccelerometerData();
	acquisition_platform.gyroscope_data = IMU_GyroscopeData(&(acquisition_platform.gyroscope_biases));
	acquisition_platform.magnetometer_data = IMU_MagnetometerData();

	acquisition_platform.global_seq++;
}

void AcquisitionPlatform_setPeriodicIMUupdate(uint16_t frequency) {
  uint16_t period_ms = 1000 / frequency; //from a frequency in Hz, we get a period in millisecs
  
  // configure timer1, prescaler : 1024, CTC (Clear Timer on Compare match)
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS12); 
  
  /*
	 * cpu frequency 16MHz = 16.000.000 Hz
	 * prescaler 1024
	 *	-->> TCNT1 increased at a frequency of 16.000.000/1024 Hz = 15625 Hz
	 *	so 1 ms will correspond do 15.625 counts
	 */
  OCR1A = (uint16_t)(15.625 * period_ms);

	// timer-interrupt enabling will be executed atomically (no other interrupts)
		// and ATOMIC_FORCEON ensures Global Interrupt Status flag bit in SREG set afetrwards
		// (sei() not needed)
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
  	TIMSK1 |= (1 << OCIE1A);  // enable the timer interrupt (istruz. elementare, no interrupt)
  }
}

uint16_t AcquisitionPlatform_getGlobalSeq(void) {
	return acquisition_platform.global_seq;
}

GyroscopeCalibrationBiases AcquisitionPlatform_getGyroscopeBiases(void) {
	return acquisition_platform.gyroscope_biases;
}

AccelerometerData AcquisitionPlatform_getAccelerometer(void) {
	return acquisition_platform.accelerometer_data;
}

GyroscopeData AcquisitionPlatform_getGyroscope(void) {
	return acquisition_platform.gyroscope_data;
}

MagnetometerData AcquisitionPlatform_getMagnetometer(void) {
	return acquisition_platform.magnetometer_data;
}
