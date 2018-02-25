//I2C initialization, read and write commands

#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <stdlib.h>
#include "rom.h"
#include "sysctl.h"
#include <string.h>
#include "hw_memmap.h"
#include "fpu.h"
#include "gpio.h"
#include "interrupt.h"
#include "pin_map.h"
#include "TM4C123GH6PM.h"
#include "i2c.h"
#include "hw_types.h"


#define I2C0_BASE 0x40020000
#define ACCELEROMETER 0x1D
#define OUT_X_MSB 0x01 // address of the x direction data register
#define CTRL_REG1 0x2A
#define ACTIVE 0x01
#define FREAD 0x02
#define XYZ_DATA_CFG 0x0E
#define FFMT_CONFIG 0x15
#define FF_MT_THS 0x17
#define FF_MT_COUNT 0x18
#define FF_MT_SRC 0x16
#define CTRL_REG4 0x2D
#define CTRL_REG5 0x2E



#define GPIO_PE4_I2C2SCL        0x00041003
#define GPIO_PE5_I2C2SDA        0x00041403
#define I2C_SLAVE_ADDRESS       0x58
#define GPIO_PB3_I2C0SDA        0x00010C03
#define GPIO_PB2_I2C0SCL        0x00010803


void I2Cwrite(uint32_t base,unsigned char addr, unsigned char *data, unsigned int len);
unsigned long WaitI2CDone( unsigned int long ulBase);
void SetupI2C(void);
unsigned char I2CRead(uint32_t base,unsigned char addr,unsigned char device_register, unsigned char* data, unsigned char len);
