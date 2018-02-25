//This code is used to run the HRV Watch
//Include needed libraries
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <stdlib.h>
#include "rom.h"
#include "sysctl.h"
#include <string.h>
#include "hw_memmap.h"
#include "gpio.h"
#include "interrupt.h"
#include "pin_map.h"
#include "timer.h"
#include "hw_ints.h"
#include "TM4C123GH6PM.h"
#include "pwm.h"
#include "i2c.h"
#include "fpu.h"
#include "AFE4404_REG_SETUP_2.h"
#include "rom_map.h"
#include "hw_types.h"
#include "uart.h"
#include "hibernate.h"
#include "uartstdio.h"
#include "Algos.h"
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif
#define data_length 500 //Sample rate is 100Hz, so this will be 5 seconds of data each run
//#define p_size 10
//#define N_ 250 //use for repiratory rate. To get filter length 
// and frequency from: Fco=.442947/sqrt(N^2-1), N=sqrt(.196202+Fco^2)/Fco. Respiratory rate lies between .2-.33. F=f/fs
#define N_ 100//.003Hz
#define N 25 //use for HR  normal is 1Hz
#define row 4
#define HRV_length 60
//FATFS definitions
FRESULT fr;
FATFS fatfs;
FIL fil;

//**********Constants**********************
//int m;	
unsigned long long sec=0;
unsigned long long subsec=0;
int j=0;
int s=0;
int n=0;
bool start=false;
static signed int hrv[4]={0};
int ind=0;
float NBFS=1;
float PBFS=1;
static signed int V[2]={0};
float CI=0;
short m;
short ma;
static signed int filt_sig[5][data_length]={0};
typedef struct RR{
	signed int BW;
	signed int FM;
	signed int AM;
	signed int peak;
	signed int time;
	short counter;
	signed int rate;
	signed int trough;
	signed int average;
}respiratory;
	
//respiratory RRpeak,RRp_old,r_,R_,rate,t_,RR_BW,RR_AM,RR_FM;
respiratory RR_={0,0,0,0,0,0,10,0,0};
respiratory RR_old={0,0,0,0,0,0,10,0};
respiratory rr_={0,0,0,0,0,0,10,0};
respiratory rr_old={0,0,0,0,0,0,10,0};
respiratory RR_1={0,0,0,0,0,0,10,0,0};
respiratory RR_old1={0,0,0,0,0,0,10,0};
respiratory rr_1={0,0,0,0,0,0,10,0};
respiratory rr_old1={0,0,0,0,0,0,10,0};
respiratory RR_2={0,0,0,0,0,0,10,0,0};
respiratory RR_old2={0,0,0,0,0,0,10,0};
respiratory rr_2={0,0,0,0,0,0,10,0};
respiratory rr_old2={0,0,0,0,0,0,10,0};


typedef struct HR{
	signed int peak;
	signed int trough;
	signed int time;
	short counter;
	signed int rate;
	signed int BPM;
	signed int average;
}HEART;	
HEART HR_={10,0,0,0,60,60};
HEART HR_OLD={10,0,4000,0,60,60};
HEART HR={10,0,0,0,60,60,0};
HEART HR_old={10,0,0,0,60,60,0};

int mean=0;
signed int HRV_STATS[2][HRV_length]={0};
int SDNN_COUNT=0;
float SDNN_VALUE=0;
float RMSSD=0;
signed int HR_M[row][data_length]={0};
signed char ACC[3][data_length]={0};
signed int HRV;
static int i=0;
int value_sum=0,value_sum_=0,val_sum=0;
int rvalue_sum=0,rvalue_sum_=0,rval_sum=0;
int filter[4][N]={0};
int resp_filter[4][N_]={0};
int Index=0,Index_=0,Index_3=0;;
int resp_Index=0,resp_Index_=0,resp_Index_3=0;
signed int signal=0,signal1=0,signal2=0;
signed int xyz=127;
signed char x=127,y=127,z=127;
signed int LED_Val_1=0;
signed int LED_AMB_Val_1=0;
signed int millis=0;
signed int t_old=0;
//signed int time_t;
unsigned char i2c_buff1[4]={0};
unsigned long ulPeriod;
unsigned char temp_st[4] = { 0 };



//********************************************

//*************Functions********************
unsigned char I2CRead(signed int base,unsigned char addr,unsigned char device_register, unsigned char* data, unsigned char len);
void init_MMA8451_Accelerometer(void);
signed int two_comp_32(uint32_t input,uint8_t num_bits);
void Timer_init(void);
void HR_time(void);
void SetMode(void);
void ACCEL_INT(void);
void ConfigureUART(void);
void ADC_Ready_Int_init(void);
void ADC_Ready(void);
void SetupI2C(void);
uint8_t AFE4404_init( uint8_t address );
void HR_calibration_(void);
//void button(void);
//void Health_Button_init(void);
//void memory_init(void);
//*********************************************


//*****************************************************************************
//
// Moves to end of file for append operation
//
//*****************************************************************************
FRESULT open_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
)
{
    FRESULT fr;

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK)
            f_close(fp);
    } else{
	}
    return fr;
}


//Conversion of 32 and 8 bit 2's complement 

signed int two_comp_32(uint32_t input,uint8_t num_bits){
signed	int two_comp;
	signed int shift_in;
two_comp= input;	
	shift_in = num_bits==8? input>>7:input>>23;
	if(shift_in==1){
		two_comp=	 num_bits==8? (signed char)(~input)+1:~(input&0x1FFFFF)+1;
	return two_comp;
	}	
	else{
	return input;
	}	
}


//Initialize timers for Interupts. The timer interupt to keep track of BPM
void Timer_init(void){
	
	
	signed int Period=24000;// every millisecond the timer overflows 
	signed int Period1=2400000; //every 10 micro seconds timer overflows
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	SysCtlDelay(3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	SysCtlDelay(3);
	TimerClockSourceSet(TIMER1_BASE,SysCtlClockGet());//clock is from system
	TimerClockSourceSet(TIMER2_BASE,SysCtlClockGet());//clock is from system
	TimerDisable(TIMER1_BASE,TIMER_A);
	TimerDisable(TIMER2_BASE,TIMER_A);
	TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC);
	TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER1_BASE,TIMER_A,Period);//this is for 100 msec interupt counter
	TimerLoadSet(TIMER2_BASE,TIMER_A,Period1);//this is for 100 msec interupt counter
	TimerLoadSet(TIMER1_BASE, TIMER_A, Period -1);
	TimerLoadSet(TIMER2_BASE, TIMER_A, Period1 -1);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);  
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);  
  TimerIntRegister(TIMER1_BASE, TIMER_A, HR_time);
	TimerIntRegister(TIMER2_BASE, TIMER_A, ACCEL_INT);
	TimerEnable(TIMER1_BASE,TIMER_A);
	TimerEnable(TIMER2_BASE,TIMER_A);
}	

//Interupt Handler for HR counter 
// Creates counter every millisecond and counter increses
void HR_time(void)
{
  signed int status=0;
  status = TimerIntStatus(TIMER1_BASE,true);
  TimerIntClear(TIMER1_BASE,status);
  millis++;
}

//Accelerometer Interupt, check if data is in buffer before reading out
void ACCEL_INT(void){
	signed int status=0;
	status = TimerIntStatus(TIMER2_BASE,true);
  TimerIntClear(TIMER2_BASE,status);
	if(I2CRead(I2C0_BASE,0x1D,0x00,i2c_buff1, 1)==255){
	xyz=get_xyz_data();
	x=two_comp_32((uint8_t)(xyz>>16),8);
	y=two_comp_32((uint8_t)(xyz>>8)&0x00FF,8);
	z=two_comp_32((uint8_t)xyz,8);	
	}
}
	
//Interupt Should trigger every 10000 us with PRF=39999
void ADC_Ready_Int_init(void){
		GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_1);        // Disable interrupt 
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);      // Clear pending interrupts 
    GPIOIntRegister(GPIO_PORTE_BASE, ADC_Ready);     // Register our handler function 
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_RISING_EDGE); // Configure  rising edge trigger
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_1);     // Enable interrupt f
}
/* Button to record moments of noticeable stress
void Health_Button_init(void){
		GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_6);        // Disable interrupt 
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);      // Clear pending interrupts 
    GPIOIntRegister(GPIO_PORTD_BASE, button);     // Register our handler function 
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6,GPIO_RISING_EDGE); // Configure  rising edge trigger
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_6);     // Enable interrupt f
}

void button(void){
	if(GPIOIntStatus(GPIO_PORTD_BASE,false) & GPIO_PIN_6 ){
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);
//UARTprintf("Button Pushed\n");
}
}
*/

void system_setup( void )
{
		//Set System address
     uint8_t address = 0x58;	 
		//Start Comm
		 SetupI2C();		 
		//Enable all GPIO Pins
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		 SysCtlDelay(3);
		 GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
	   GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6);
		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		 SysCtlDelay(3);
		 GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
		 GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,0);
		 SysCtlDelay(1000);//delay for roughly 40us
		 GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,GPIO_PIN_2);
		 SysCtlDelay(30000);//delay for 1.25us after reset prior to I2c startup
		//Initialize AFE4404 with all settings
		 AFE4404_init( address);


}

void PWM_init(){

  //Configure PWM Clock to match system
		SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
		SysCtlDelay(3);
		ulPeriod=SysCtlPWMClockGet();
    // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	  SysCtlDelay(3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins
    SysCtlDelay(3);
    //Configure PF1,PF2,PF3 Pins as PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
	  GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_2);
    //Configure PWM Options
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 6);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,6/2); //set 20% duty cycle
    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    // Turn on the Output pins
		 PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
		
//PWM init end

}

//Configure UART for Serial data. Used for initial testing
/*
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	  SysCtlDelay(3);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	  SysCtlDelay(3);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    //
   // UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
		UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
		SysCtlDelay(3);

    //UART_CLOCK_SYSTEM
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 24000000);}
*/
//Calibration Routine
void HR_calibration_(void){
/* This should be done at start up
Set timing for sampling pulse duration ?? this is for power saving
Set TI Gain to 1M ohm set averaging mode between 4-8
Set LED to desired lowest current settings
*/
int T_GAIN=7;
int current=15; //Current is set to 25mA
int T_count=0;
int bit_offset=0;
int bit_sign=1;
int bit_offset_2=0;
int bit_sign_2=1;
int average=0;
int non_converge=0;
int count=0;	
int buff=0;
//Initially calculate average PPG value
while (count<50){
	buff+=two_comp_32(hr_get_led1_val(),32);
	count++;	
}
average=buff/count;	
count=0;
buff=0;
current=15;
//While the PPG value is not within +-1V range for ADC for PPG 1 Channel
while((abs(average) > 1677720) ){ //2097151=1.2V, 1677720=1.0V
//If value is not within range after 5 iterations, update AFE settings
	if(non_converge>5){
			bit_offset++;
			T_count++;
			hr_set_offdac_settings(0,3,bit_sign,bit_offset,bit_sign,bit_offset,1,7);		
			SysCtlDelay(30000);
			non_converge=0;
	}
	//If IDAC has stepped through all values, flip bit to change sign
	if(bit_offset==7){
		bit_offset=0;
		bit_sign=1-bit_sign;
		
	}			
	//If all IDAC values have stepped through update gain and current 
	if(T_count==30 && T_GAIN>0){
	  T_GAIN--;	
		current=current-2;
		current<=1?current=15:current;	
	   hr_set_tia_gain( false, T_GAIN, 0);
		 hr_set_led_currents( current,current,0);
		 SysCtlDelay(30000);
		 T_count=0;
	}			
	T_GAIN==0 ? T_GAIN=7:T_GAIN;

	//Reconfigure PPG value
		while (count<50){
			buff+=two_comp_32(hr_get_led1_val(),32);
			count++;
		}
		
	non_converge++;
	average=buff/count;	
	count=0;
	buff=0;
}
non_converge=0;
count=0;
buff=0;
//check for PPG 2 channel
while (count<50){
	buff+=two_comp_32(hr_get_led2_val(),32);
	count++;	
}
average=buff/count;	
while((abs(average) > 1677720) ){ //2097151=1.2V, 1677720=1.0V
	if(non_converge>5){
		bit_offset_2++;
		hr_set_offdac_settings(bit_sign_2,bit_offset_2,bit_sign,bit_offset,bit_sign,bit_offset,bit_sign_2,bit_offset_2);	
		SysCtlDelay(30000);
		non_converge=0;
	}
	if(bit_offset_2==15){
	bit_offset_2=0;
		bit_sign_2=1-bit_sign_2;
		
	}			
		while (count<50){
			buff+=two_comp_32(hr_get_led2_val(),32);
			count++;
		}
	
	  non_converge++;
	  average=buff/count;	
		count=0;
		buff=0;
	
}			
}

//Interupt to read LED Values for each timing routine
void ADC_Ready(void){
		if(GPIOIntStatus(GPIO_PORTE_BASE,false) & GPIO_PIN_1 ){
	//Tristate Clock during Power Down
	PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
	SysCtlDelay(115);
	PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
						
	GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);
			LED_Val_1=two_comp_32(hr_get_led1_val(),32);
	if(abs(LED_Val_1)> 1677720 ){	 				
	SysCtlReset();
	}	
		
 else if (n%1==0){			//downsample using n to save computation 
  HR_M[0][i]=(two_comp_32(hr_get_led1_amb1_val(),32)+two_comp_32(hr_get_led2_amb2_val(),32))/2;
	HR_M[1][i]=millis;//Get Timer time
	HR_M[2][i]=HibernateRTCGet();//Get RTC time
	HR_M[3][i]=HibernateRTCSSGet();//Get RTC time
	ACC[0][i]=x*.0156; //Convert to g's
	ACC[1][i]=y*.0156;
	ACC[2][i]=z*.0156;
	
	i++;	
	n++;
}
else{
n++;
}
}
	}	



//****************Start Program****************************
int main(void){
	//Lazy stacking for efficiency
ROM_FPULazyStackingEnable();
	
    //Set the clock
SysCtlClockSet(SYSCTL_SYSDIV_1| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);;  // SET SYSTEM CLOCK FOR 24Mhz, off of PLL
	
SysCtlDelay(3);

   //enable the hibernation module for RTC
  SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);

  //SYSCTL_OSC_EXT32 is the rate of the clock supplied to the Hibernation
  HibernateEnableExpClk(SYSCTL_OSC_EXT32); 
 HibernateClockConfig(HIBERNATE_OSC_HIGHDRIVE);//Highdrive is 24pF cap on 32kHz line
  HibernateRTCDisable();

   //set the value to 0
   HibernateRTCSet(0);
   HibernateRTCEnable();

   //get starting values
 sec = HibernateRTCGet();
 subsec =  HibernateRTCSSGet(); 
 
ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

//SD Card Initialization
fr = f_mount(&fatfs, "", 0);	
	
//ConfigureUART();    			
	
//initialize pwm for clock source
PWM_init();

//Setup System
system_setup();	

//IntPrioritySet(INT_GPIOD_TM4C123,3);
IntPrioritySet(INT_TIMER1A_TM4C123, 1);
IntPrioritySet(INT_TIMER1B_TM4C123, 2);
IntPrioritySet(INT_GPIOE_TM4C123,0);

//Set Accel Mode
SetMode();	

HR_calibration_();
ADC_Ready_Int_init();
Timer_init();
//Health_Button_init();
HR_OLD.peak=1000;


    while(1){
	//While the physical activity count is greater than 10, do nothing to negate possible motion artefacts	
while (physical_activity_count(two_comp_32(x,8),two_comp_32(y,8),two_comp_32(z,8))>10){
	TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);  
	  GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_1);  
		i=i;
	SysCtlDelay(3000000);
	}
   
			GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_1);  
			TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); 
		
if(i>=data_length){
	i=0;
GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_1);        // Disable interrupts during algorithm comp
TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);  
TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);  
	
	for(j=1;j<data_length;j++){
		//Start of Filtering, shift out value and shift in new for running average, for every N values
	value_sum-=filter[0][Index];
  filter[0][Index]=HR_M[0][j]-HR_M[0][j-1]; //Difference HPF Filter
	//filter[0][Index]=(HR_M[0][j]-HR_M[0][j-2])/2; High Pass Filter with diff frequency response
  value_sum+=filter[0][Index];
  Index++;
  if (Index>=N){
    Index=0;
	}
	filt_sig[0][j]=value_sum/N;
	//2nd order running average
	value_sum_-=filter[2][Index_];
	filter[2][Index_]=filt_sig[0][j];
  value_sum_+=filter[2][Index_];
	
  Index_++;
  if (Index_>=N){
    Index_=0;
	}
	filt_sig[0][j]=value_sum_/N;

	//3rd order running average, Gaussian Filter
	val_sum-=filter[3][Index_3];
  filter[3][Index_3]=filt_sig[0][j];
  val_sum+=filter[3][Index_3];
Index_3++;
  if (Index_3>=N){
    Index_3=0;
	}
	filt_sig[0][j]=val_sum/N;
	
	
	//respiration Filtering start, to get baseline wander
	
	rvalue_sum-=resp_filter[0][resp_Index];
  resp_filter[0][resp_Index]=HR_M[0][j]-HR_M[0][j-1];
  rvalue_sum+=resp_filter[0][resp_Index];
  resp_Index++;
  if (resp_Index>=N_){
    resp_Index=0;
	}
	filt_sig[3][j]=rvalue_sum/N_;
	rvalue_sum_-=resp_filter[1][resp_Index_];
	 resp_filter[1][resp_Index_]=filt_sig[3][j];
  rvalue_sum_+=resp_filter[1][resp_Index_];	
  resp_Index_++;
  if (resp_Index_>=N_){
    resp_Index_=0;
	}
	filt_sig[3][j]=rvalue_sum_/N_;
	
	rval_sum-=resp_filter[2][resp_Index_3];
  resp_filter[2][resp_Index_3]=filt_sig[3][j];
  rval_sum+=resp_filter[2][resp_Index_3];
	resp_Index_3++;
  if (resp_Index_3>=N_){
    resp_Index_3=0;
	}
	filt_sig[3][j]=rval_sum/N_;
	}

if(start==true){	//allows for filtering to get established

for(j=1;j<data_length;j++){ //begin HR,HRV and RR comp
	//RLS Filtering using constants not matrix mults
	signal=RLS(filt_sig[0][j],ACC[0][j]);
	signal1=RLS(signal,ACC[1][j]);
	signal2=RLS(signal1,ACC[2][j]);
	
	//Peak detection for HR
HR.peak=(Peak_Detect((filt_sig[0][j]),1));
if(HR_OLD.peak!=HR.peak ){
	HR.counter++;	
	hrv[ind]=HR_M[1][j];		
	ind++;	
		//HRV, SDNN and RMSSD	
	if((HR_M[1][j]-t_old)>450&&(HR_M[1][j]-t_old)<2050){
			HRV_STATS[0][SDNN_COUNT]=HR_M[1][j]-t_old;
			t_old=HR_M[1][j];			
			mean+=HRV_STATS[0][SDNN_COUNT];
			SDNN_COUNT++;
		if(SDNN_COUNT>=HRV_length){
		mean=mean/SDNN_COUNT;
			for(s=1;s<HRV_length;s++){
				SDNN_VALUE+=(HRV_STATS[0][s]-mean)*(HRV_STATS[0][s]-mean);		
				RMSSD+=(HRV_STATS[0][s]-HRV_STATS[0][s-1])*(HRV_STATS[0][s]-HRV_STATS[0][s-1]);
			}
		SDNN_VALUE=sqrt(SDNN_VALUE/(HRV_length-1));
		RMSSD=sqrt(RMSSD/(HRV_length-1));
		SDNN_COUNT=0;
		mean=0;
		SDNN_VALUE=0;
		RMSSD=0;
		}
	}
		else{
		t_old=HR_M[1][j];
		}
//PNN50 count for HRV
			if (ind==4){
			 // HRV=(abs(abs(hrv[0]-hrv[1])-abs(hrv[1]-hrv[2]))+abs(abs(hrv[2]-hrv[3])-abs(hrv[1]-hrv[2])))/2;			
			  abs(hrv[0]-hrv[1])>abs(hrv[1]-hrv[2])?V[0]=1:0;
				abs(hrv[0]-hrv[1])<abs(hrv[1]-hrv[2])?V[0]=2:0;
				abs(hrv[0]-hrv[1])==abs(hrv[1]-hrv[2])?V[0]=0:0;
				abs(hrv[1]-hrv[2])>abs(hrv[2]-hrv[3])?V[1]=1:0;
				abs(hrv[1]-hrv[2])<abs(hrv[2]-hrv[3])?V[1]=2:0;
				abs(hrv[1]-hrv[2])==abs(hrv[2]-hrv[3])?V[1]=0:0;
				V[0]==1 &&V[1]==1?PBFS++:0;
			  V[0]==2 &&V[1]==2?PBFS++:0;
				V[0]==1 &&V[1]==2?NBFS++:0;
				V[0]==2 &&V[1]==1?NBFS++:0;
				V[0]==0 &&V[1]==1?NBFS++:0;
				V[0]==1 &&V[1]==0?NBFS++:0;
				V[0]==0 &&V[1]==2?NBFS++:0;
				V[0]==2 &&V[1]==0?NBFS++:0;
				V[0]==0 &&V[1]==0?NBFS++:0;		
			  ind=0;
			}
		HR.average+=HR.peak;
		HR_old.counter++;
			//update peak detect steo averaged over 5 beats
		if(HR_old.counter>5){
		HR.average=abs(HR.average/HR_old.counter);
		deltad=HR.average/2;
		HR.average=0;
		HR_old.counter=0;
		}
		//Update peak detect step 
		deltad=Kalman(deltad,(HR_.rate+HR.rate)/2,1);
		HR.peak!=0?HR_OLD.peak=HR.peak:HR_OLD.peak;
		//Get HR averaged over 5 beats
			if(HR.counter>=5){
			  HR.rate=(60000*HR.counter)/(HR_M[1][j]-HR.time);
				HR.time=HR_M[1][j];
				 HR.counter=0;	
				//Thresholding for HR
				if (HR.rate<(HR_old.rate+HR_old.rate*.75)&&HR.rate>(HR_old.rate-HR_old.rate*.75)&&HR.rate>35&&HR.rate<140){
				HR_old.rate=HR.rate;
				}
					else{
							HR.rate=HR_old.rate;
					}
			  			
			}
}
//Trough detection for HR
HR_.trough=(Peak_Detect((filt_sig[0][j]),0));
	
if(HR_OLD.trough!=HR_.trough ){
		HR_.counter++;
	//Thresholding for RR frequency modulation
	if((HR_M[1][j]-HR_OLD.time)>450&&(HR_M[1][j]-HR_OLD.time)<2050){
	RR_.FM=(HR_M[1][j]-HR_OLD.time);
	HR_OLD.time=HR_M[1][j];
	}
	else{
	HR_OLD.time=HR_M[1][j];
	}
	//Baseline wander
		RR_.BW=(HR_.trough+HR.peak)/2;
	//Amplitude modulation for RR
		RR_.AM=(HR.peak-HR_.trough);
		HR_OLD.trough=HR_.trough;
	//Get HR averaged over 5 beats
			if(HR_.counter>=5){
			 HR_.rate=(60000*HR_.counter)/(HR_M[1][j]-HR_.time);
				HR_.time=HR_M[1][j];
					HR_.counter=0;
				if (HR_.rate<(HR_OLD.rate+HR_OLD.rate*.75)&&HR_.rate>(HR_OLD.rate-HR_OLD.rate*.75)&&HR_.rate>35 &&HR_.rate<140){
				HR_OLD.rate=HR_.rate;
				HR.BPM=(HR_.rate+HR.rate)/2;
				//Kalman filter HR
				HR_.BPM=Kalman(deltad,(HR_.rate+HR.rate)/2,0);
				}
					else{
							HR_.rate=HR_OLD.rate;
					}
					//write HR, RR, HRV and time to SD card
			fr = open_append(&fil, "logfile.csv");
			f_printf(&fil,"%d,%d,%d,%d,%d\n",HR_.BPM,rr_.average,(int)CI,HR_M[2][j],HR_M[3][j]);
			fr = f_close(&fil);
				
			}
}

	//The following peak and trough detects the respiratory rate signals to acquire RR
rr_.peak=(Peak_Detect2(RR_.FM,1));
if(rr_old.peak!=rr_.peak  ){
		rr_.counter++;
		rr_old.peak=rr_.peak;
	  RR_.average+=rr_.peak;
		rr_old.counter++;
	if(rr_old.counter>2){
			deltad2=abs(RR_.average/(rr_old.counter*10));
		RR_.average=0;
		rr_old.counter=0;
		}
			if(rr_.counter>=3){
			 rr_.rate=(60000*rr_.counter)/(HR_M[1][j]-rr_.time);
				rr_.time=HR_M[1][j];
			  rr_.counter=0;			
				
			}
}

RR_.trough=(Peak_Detect2(RR_.FM,0));
if(RR_old.trough!=RR_.trough  ){
		RR_.counter++;
		RR_old.trough=RR_.trough;
			if(RR_.counter>=3){
			 RR_.rate=(60000*RR_.counter)/(HR_M[1][j]-RR_.time);
				RR_.time=HR_M[1][j];
			  RR_.counter=0;
					RR_.rate=(RR_.rate+rr_.rate)/2;
				
			}
}



rr_1.peak=(Peak_Detect3(RR_.BW,1));
if(rr_old1.peak!=rr_1.peak  ){
		rr_1.counter++;
		rr_old1.peak=rr_1.peak;
	  RR_1.average+=rr_1.peak;
		rr_old1.counter++;
	if(rr_old1.counter>2){
		deltad3=abs(RR_1.average/(rr_old1.counter*8));
		RR_1.average=0;
		rr_old1.counter=0;
		}
			if(rr_1.counter>=3){
			 rr_1.rate=(60000*rr_1.counter)/(HR_M[1][j]-rr_1.time);
				rr_1.time=HR_M[1][j];
			  rr_1.counter=0;			
				
			}
}

RR_1.trough=(Peak_Detect3(RR_.BW,0));
if(RR_old1.trough!=RR_1.trough  ){
		RR_1.counter++;
		RR_old1.trough=RR_1.trough;
			if(RR_1.counter>=3){
			 RR_1.rate=(60000*RR_1.counter)/(HR_M[1][j]-RR_1.time);
				RR_1.time=HR_M[1][j];
			  RR_1.counter=0;
					RR_1.rate=(RR_1.rate+rr_1.rate)/2;
				//Kalman(deltad,(RR_.rate+rr_.rate)/2,1);
			}
}


rr_2.peak=(Peak_Detect4(RR_.AM,1));
if(rr_old2.peak!=rr_2.peak  ){
		rr_2.counter++;
		rr_old2.peak=rr_2.peak;
	  RR_2.average+=rr_2.peak;
		rr_old2.counter++;
	if(rr_old2.counter>2){
		deltad4=abs(RR_2.average/(rr_old2.counter*8));
		RR_2.average=0;
		rr_old2.counter=0;
		}
			if(rr_2.counter>=3){
			 rr_2.rate=(60000*rr_2.counter)/(HR_M[1][j]-rr_2.time);
				rr_2.time=HR_M[1][j];
			  rr_2.counter=0;			
				
			}
}

RR_2.trough=(Peak_Detect4(RR_.AM,0));
if(RR_old2.trough!=RR_2.trough  ){
		RR_2.counter++;
		RR_old2.trough=RR_2.trough;
			if(RR_2.counter>=3){
			 RR_2.rate=(60000*RR_2.counter)/(HR_M[1][j]-RR_2.time);
				RR_2.time=HR_M[1][j];
			  RR_2.counter=0;
					RR_2.rate=(RR_2.rate+rr_2.rate)/2;
				//Kalman(deltad,(RR_.rate+rr_.rate)/2,1);
			}
}
//Updates RR based on negating the largest outlier
rr_.average=(RR_2.rate+RR_1.rate+RR_.rate)/3;
m=abs(RR_2.rate-rr_.average)>abs(RR_1.rate-rr_.average)?RR_2.rate:RR_1.rate;
ma=abs(RR_.rate-rr_.average)>abs(m-rr_.average)?RR_.rate:m;
rr_.average=((rr_.average*3)-(ma))/2;

if (rr_.average>5 && rr_.average<18){
	//Kalman Filter RR rate 
	rr_.average=Kalman2(deltad3,rr_.average,0);
rr_old.average=rr_.average;
}
else{
	rr_.average=rr_old.average;
}

}
//Central Index to compute PNN50
CI=(PBFS/(PBFS+NBFS))*100;
TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);  
TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);  
GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_1);     // Enable interrupt 

}
start=true;
i=0;
}

}

}







