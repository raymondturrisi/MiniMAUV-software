//cpp file required for pt_sensors.cpp, found on [https://discuss.bluerobotics.com/t/bar30-pressure-sensor-with-beaglebone-black/404/4]
//Modified by Raymond Turrisi

#include <iostream>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//#include <limits.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "Bar30.h"

#define MS5837_ADDR               0x76
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

  int file;
  int adapter_nr = 2;
  char filename[40];

MS5837::MS5837() {
	fluidDensity = 1029;

	sprintf(filename,"/dev/i2c-1");
  	file = open(filename, O_RDWR);//, O_NONBLOCK,O_NDELAY);
  	if (file < 0)
  	{
		printf("Failed to open the bus.");
    		/* ERROR HANDLING; you can check errno to see what went wrong */
   	 	exit(1);
  	}

  	//int addr = 0x76; /* The I2C address */

  	if (ioctl(file, I2C_SLAVE, MS5837_ADDR) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
    		/* ERROR HANDLING; you can check errno to see what went wrong */
    		exit(1);
  	}
}

void MS5837::init() {

	uint8_t buffer[2];
	int n_writ, n_read;
	memset(buffer,'\0',2);

	// Reset the MS5837, per datasheet

	buffer[0] = MS5837_RESET;
	n_writ = write(file,buffer,1);
	if( n_writ < 1)
	{
	        /* ERROR HANDLING: i2c transaction failed */
        	printf("Failed to write to the i2c bus.\n");
	}

	printf("Waiting for PRESSURE SENSOR to initialize...\n\n");

	// Wait for reset to complete
	usleep(100000);
	printf("PRESSURE SENSOR initialized!\n\nReading and storing calibration data...\n\n");


	memset(buffer,'\0',2);
	// Read calibration values and CRC
	for ( uint8_t i = 0 ; i < 7 ; i++ )
	{
		memset(buffer,'\0',2);
		buffer[0] = MS5837_PROM_READ+(i*2);
		n_writ = write(file,buffer,1);
		n_read = read(file,buffer,2);
		if (n_read != 2)
		{
            		/* ERROR HANDLING: i2c transaction failed */
            		printf("Failed to read from the i2c bus.\n");
			C[i] = 0;
        	}
		else
		{
			C[i] = (((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
			printf("C[%i] = %i\n",i,C[i]);
		}

	}

	printf("Calibration data stored. Checking CRC...\n\n");
	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);

	printf("CRC Recv'd:  %i, CRC Calc:  %i\n\n",crcRead,crcCalculated);

	if ( crcCalculated == crcRead )
	{
		// Success
		printf("Calibration Values CRC Check Success!\n");
	}
	else
	{
		// Failure - try again?
		printf("Calibration Values CRC Check FAILURE!\n");
	}

	printf("Setup complete!\n\n");
}

void MS5837::setFluidDensity(float density) {
	fluidDensity = density;
}

void MS5837::read_pressure() {

	uint8_t buffer[3];
	memset(buffer,'\0',3);
	int n_writ, n_read;
	n_writ = 0;
	// Request D1 conversion
	buffer[0] = MS5837_CONVERT_D1_8192;
	n_writ = write(file,buffer,1);
	if (n_writ != 1)
	{
        	/* ERROR HANDLING: i2c transaction failed */
        	printf("Failed to write 1 to the i2c bus.\n");
        }

	usleep(20000); // Max conversion time per datasheet
	n_writ = 0;
	buffer[0] = MS5837_ADC_READ;
	n_writ = write(file,buffer,1);
	if (n_writ != 1)
	{
        	/* ERROR HANDLING: i2c transaction failed */
        	printf("Failed to write 2 to the i2c bus.\n");
        }
	D1 = 0;
	n_read = read(file,buffer,3);
	if (n_read != 3)
	{
       		/* ERROR HANDLING: i2c transaction failed */
       		printf("Failed to read 1 from the i2c bus.\n");
       	}
	else
	{
		D1 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
	}

	// Request D2 conversion
	buffer[0] = MS5837_CONVERT_D2_8192;
	n_writ = write(file,buffer,1);
	if (n_writ != 1)
	{
        	/* ERROR HANDLING: i2c transaction failed */
        	printf("Failed to write 3 to the i2c bus.\n");
        }

	usleep(20000); // Max conversion time per datasheet

	buffer[0] = MS5837_ADC_READ;
	n_writ = write(file,buffer,1);
	if (n_writ != 1)
	{
        	/* ERROR HANDLING: i2c transaction failed */
        	printf("Failed to write 4 to the i2c bus.\n");
        }
	D2 = 0;
	n_read = read(file,buffer,3);
	if (n_read != 3)
	{
       		/* ERROR HANDLING: i2c transaction failed */
       		printf("Failed to read 2 from the i2c bus.\n");
       	}
	else
	{
		D2 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
	}

	//printf("D1 = %i,D2 = %i\n\n",D1,D2);
	calculate();
}

void MS5837::readTestCase() {
	C[0] = 0;
	C[1] = 34982;
	C[2] = 36352;
	C[3] = 20328;
	C[4] = 22354;
	C[5] = 26646;
	C[6] = 26146;
	C[7] = 0;

	D1 = 4958179;
	D2 = 6815414;

	calculate();
}

void MS5837::calculate() {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation

	int32_t dT;
	int64_t SENS;
	int64_t OFF;
	int32_t SENSi;
	int32_t OFFi;
	int32_t Ti;
	int64_t OFF2;
	int64_t SENS2;

	// Terms called
	dT = D2-uint32_t(C[5])*256l;
	SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
	OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;


	//Temp and P conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
	P = (D1*SENS/(2097152l)-OFF)/(8192l);

	//Second order compensation
	if((TEMP/100)<20){         //Low temp
		Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
		OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
		SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
		if((TEMP/100)<-15){    //Very low temp
			OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
			SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
		}
	}
	else if((TEMP/100)>=20){    //High temp
		Ti = 2*(dT*dT)/(137438953472LL);
		OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
		SENSi = 0;
	}

	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;

	TEMP = (TEMP-Ti);
	P = (((D1*SENS2)/2097152l-OFF2)/8192l);
}

float MS5837::pressure(float conversion) {
	return P/10.0f*conversion;
	std::cout << "pressure " << P/10.0f*conversion << '\n';
}

float MS5837::temperature() {
	return TEMP/100.0f;
	std::cout << "temp " << TEMP/100.0f << '\n';
}

float MS5837::depth() {
	return (pressure(MS5837::Pa)-101300)/(fluidDensity*9.80665);
}

float MS5837::altitude() {
	//return (1-pow((pressure()/1019.90),.190284))*145366.45*.3048;

	return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}


uint8_t MS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return (n_rem ^ 0x00);
}
