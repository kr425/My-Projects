//Accel Functions and setup

#include <stdint.h>
#include "HRV_I2C_.h"
uint16_t* data;
uint8_t slave_addr=0x1D;
//uint8_t slave_addr=0x1C;
uint16_t XMSB=0x01;
uint16_t YMSB=0x03;
uint16_t ZMSB=0x05;
//uint16_t CTRL_REG1=0x2A;
uint16_t XYZ_Data_config=0x0E;
unsigned char F_READ=0x09;
unsigned char i2c_buff[2]={0};
	
	
//Put Accel into standby mode for register writes
void standby(uint8_t slave_addr,unsigned char *data){

	I2Cwrite(I2C0_BASE,slave_addr, data, 2);
		
}
	
//Put accel into active mode
void active(int slave_addr,unsigned char *data){
	I2Cwrite(I2C0_BASE,slave_addr, data, 2);
}

void SetMode(void){
//standby	
	i2c_buff[0]=0x2A;
	i2c_buff[1]=0x00;
	standby(slave_addr,i2c_buff);
//set Read bit for 8 bit resolution
	i2c_buff[0]=0x2A;
	i2c_buff[1]=0x02;
	I2Cwrite(I2C0_BASE,slave_addr, i2c_buff, 2);
	//set HPF data
	i2c_buff[0]=0x0E;
	i2c_buff[1]=0x10;
	I2Cwrite(I2C0_BASE,slave_addr, i2c_buff, 2);
	//set HPF cut off
	i2c_buff[0]=0x0F;
	i2c_buff[1]=0x03; //2 Hz HPF
	I2Cwrite(I2C0_BASE,slave_addr, i2c_buff, 2);
//make active
	i2c_buff[0]=0x2A;
	i2c_buff[1]=0x01;
	active(slave_addr,i2c_buff);
//F_read is zero for proper auto increment address
	i2c_buff[0]=F_READ;
	i2c_buff[1]=0x0000;
	I2Cwrite(I2C0_BASE,slave_addr, i2c_buff, 2);
}	

uint32_t get_xyz_data( void ){
	
		uint8_t data[4]={0};
		
		uint32_t ret_value=0;
		
		I2CRead(I2C0_BASE,0x1D,0x00,data, 4);
		
		ret_value |= (uint32_t)data[0] <<24;//register 0 
		ret_value |= (uint32_t)data[1] <<16;//XMSB
		ret_value |= (uint32_t)data[2] <<8;//YMSB reg
		ret_value |= (uint32_t)data[3] <<0;//ZMSB reg

return ret_value;
}









