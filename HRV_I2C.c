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
#include "HRV_I2C_.h"
#include "uart.h"
#include "uartstdio.h"


#define GPIO_PE4_I2C2SCL        0x00041003
#define GPIO_PE5_I2C2SDA        0x00041403
#define I2C_SLAVE_ADDRESS       0x58
uint32_t k;
//to check if reading and writing is finished
unsigned long WaitI2CDone( unsigned int long ulBase){
    // Wait until done transmitting
    while( I2CMasterBusy(I2C2_BASE));
    // Return I2C error code
    return I2CMasterErr( I2C2_BASE);
}

//Setup I2C2 for Initialization
void SetupI2C(void){
    
     // I2C Setting
     
	 SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C2);
	SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C0);
	//reset I2C module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
   
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

// Configure the pin muxing for I2C2 functions on port E4 and E5.
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4); 
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
		GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); 
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
  
   // Enable and initialize the I2C2 master module.  Use the system clock for
	// the I2C2 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.

    I2CMasterInitExpClk( I2C2_BASE, SysCtlClockGet(), true);
		I2CMasterInitExpClk( I2C0_BASE, SysCtlClockGet(), true);
//clear I2C FIFOs	
    SysCtlDelay(10000);
}




unsigned char I2CRead(uint32_t base,unsigned char addr,unsigned char device_register, unsigned char* data,unsigned char len)
{
    if (len < 1) { // Assume I2C Recieving will always return data
        return -1;
		}
    // Set address and register to write to *****This may not be needed for initialization*******
    I2CMasterSlaveAddrSet(base, addr, false);
		I2CMasterDataPut(base, device_register);
		I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_SEND);
      while(I2CMasterBusy(base));
		k=I2CMasterErr( base);
	
		//******************************************************************************************
		
		I2CMasterSlaveAddrSet(base, addr, true);
    // Check to see if pointer is to an array
    if (len == 1){
        I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_RECEIVE);
        k=	WaitI2CDone(base);
		
        *data = I2CMasterDataGet(base);
        return *data;
			
    }

    // Begin reading consecutive data
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_START);
		WaitI2CDone(base);

    *data = I2CMasterDataGet(base);// *data is the actual value in memory
		
    len--;
    data++;//address ++, this goes to the next address and stores the *data there
		
    // Continue reading consecutive data
    while(len > 1){
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(base));
		k=I2CMasterErr( base);
			
        *data = I2CMasterDataGet(base);	
        len--;
        data++;
		
    }

    // Read last character of data  
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    	while(I2CMasterBusy(base));
		k=I2CMasterErr( base);
			
	
    *data = I2CMasterDataGet(base);

		return *data;
}



void I2Cwrite(uint32_t base, uint8_t addr, unsigned char *data, unsigned int len)
{
	
    I2CMasterSlaveAddrSet(base, addr, false);
			
    I2CMasterDataPut(base, *data);
	
    if (len == 1){
        I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_SEND);
        while(I2CMasterBusy(base));
        return;
    }

    // Start sending consecutive data
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(base));
		

    len--;
    data++;

    // Continue sending consecutive data
    while(len > 1){
        I2CMasterDataPut(base, *data);
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_CONT);
        while(I2CMasterBusy(base));
			k=I2CMasterErr( base);
	
        len--;
        data++;
    }

 // Send last piece of data
    I2CMasterDataPut(base, *data);
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(base));
		k=I2CMasterErr( base);
	
}













