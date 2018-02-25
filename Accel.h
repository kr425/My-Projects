//Accelerometer Header File

#include <stdint.h>
#include "HRV_I2C_.h"
unsigned int data;

uint16_t rawX,rawY, rawZ;
void init_MMA8451_Accelerometer(void);
uint8_t getAccelerometer_ID(void);//Don't know if this is necesary, reg -> 0x0D-0x0E
void SetPowerMode(unsigned char powerMode);
void SetMode(void);
void standby(void);
void active(void);
//int getAccelerationData(void);
uint16_t getAcceleration_rawX();
uint16_t getAcceleration_rawY();
uint16_t getAcceleration_rawZ();
//signed int getAcceleration_X();
//signed int getAcceleration_Y();
//signed int getAcceleration_Z();
//signed int getPitch();
//signed int getRoll();


