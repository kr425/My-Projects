//Initialization of TI AFE4404 HR Analog Front end

#include <stdint.h>
#include "AFE4404_REG_SETUP_2.h"
#include "HRV_I2C_.h"
#define I2C_SLAVE_ADDRESS       0x58


uint8_t AFE4404_init( uint8_t address)
{
    //hr_hal_init( address );
	  SetupI2C();
	//enable software Reset and Disable counter reset and disable register readout mode
    hr_set_settings( );
	
		// The following is for NO DIVISION OF CLOCK TO TIMING ENGINE CLOCK (CLKDIV_PRF = 1)

    hr_set_led2_start_end( 0, 399 );
    hr_set_led2_sample_start_end( 80, 399 );
    hr_set_adc_reset0_start_end( 401, 407 );
    hr_set_led2_convert_start_end( 408, 1467 );
    hr_set_led3_start_stop( 400, 799 );
    hr_set_led3_sample_start_end( 480, 799 );
    hr_set_adc_reset1_start_end( 1469, 1475 );
    hr_set_led3_convert_start_end( 1476, 2535 );
    hr_set_led1_start_end( 800, 1199 );
    hr_set_led1_sample_start_end( 880, 1199 );
    hr_set_adc_reset2_start_end( 2537, 2543 );
    hr_set_led1_convert_start_end( 2544, 3603 );
    hr_set_amb1_sample_start_end( 1279, 1598 );
    hr_set_adc_reset3_start_end( 3605, 3611 );
    hr_set_amb1_convert_start_end( 3612, 4671 );
    hr_set_pdn_cycle_start_end( 5471, 39199 );
    hr_set_prpct_count( 39999 );//Clock timing for CLKDIV_PRF  = 1
		
    //ADC Average Set to 3 for Timing Control
    hr_set_timer_and_average_num( true, 7 );
		
    hr_set_seperate_tia_gain( true, 7, 0 ); //Allows for two sets of TIA gain settings for the channels, when true args 2 and 3 control cf2 and rf2 for LED2,3
		
    hr_set_tia_gain( false, 7, 0);// set TIA resistor value and capacitor 
/* ********************************************************************
 * LED RF and CF Bit values                     *
 * ****************************************************************** *
 *             Register Values      |              Rf k ohm      
 *                   0              |              500                  
 *                   1              |              250                
 *                   2              |              100                 
 *                   3              |              50                 
 *                   4              |              25                
 *                   5              |              10
 *									 6														 1M
										 7														 2M 	
 **********************************************************************
 * ****************************************************************** *
 *                  Cap Values      |              Cf pf      
 *                   0              |              5                  
 *                   1              |              2.50                
 *                   2              |              10                 
 *                   3              |              7.5                 
 *                   4              |              20                
 *                   5              |              17.5
 *									 6														 25
										 7														 22.5 	
 **********************************************************************/


		//100 mA LEDS ,GREENMAX=25 REDMAX=40 IRMAX= 60 
    hr_set_led_currents( 15,15,0);

/* ********************************************************************
 * LED current control is a 24 bit register where                     *
 * LED1: bits 0-5 LED2: bits 6-11 LED3: bits 12-17 and the rest are 0 *
 * ****************************************************************** *
 * LED1, LED2, LED3 Register Values | LED Current Setting (mA)        *
 *                   0              |              0                  *
 *                   1              |             0.8                 *
 *                   2              |             1.6                 *
 *                   3              |             2.4                 *
 *                  ...             |             ...                 *
 *                   63             |              50                 *
 **********************************************************************/
		
		
		
		
		
		//division ratio for clk_ext div=1 4MHZ
		hr_set_inp_inn_settings(5);
		
		/*
		Transmitter disabled, 
		LED 0-100, 
		ADC pwrd dwn, 
		Ext clk, 
		TIA NPDn, 
		RADC NPDn, 
		Normal Mode, 
		Normal Mode
		*/
		
hr_set_dynamic_settings( 0,1,1,0,0,0,0,0 );	
	
	
	
hr_set_offdac_settings(0,0,0,0,0,0,0,0);	
/*
Set DAC Offsets to be LED 2 > -1.4,LED3 > -1.87 and LED1 > -.93 
Order is LED2(pol,dac_v),Ambient(pol,dac_v),Led1(pol,dac_v),LED3(pol,Dac_v)
* **************************************************************
 *  I_OFFDAC Register Settings                                  *
 * **************************************************************
 * * Reg. Settings | Offset Cancellation | Offset Cancellation  |
 * *               |   POL_OFFDAC = 0    |   POL_OFFDAC = 1     |
 * **************************************************************
 *        0        |          0          |           0          *
 *        1        |       0.47          |       –0.47          *
 *        2        |       0.93          |       –0.93          *
 *        3        |        1.4          |        –1.4          *
 *        4        |       1.87          |       –1.87          *
 *        5        |       2.33          |       –2.33          *
 *        6        |        2.8          |        –2.8          *
 *        7        |       3.27          |       –3.27          *
 *        8        |       3.73          |       –3.73          *
 *        9        |        4.2          |        –4.2          *
 *       10        |       4.67          |       –4.67          *
 *       11        |       5.13          |       –5.13          *
 *       12        |        5.6          |        –5.6          *
 *       13        |       6.07          |       –6.07          *
 *       14        |       6.53          |       –6.53          *
 *       15        |          7          |          –7          *
 ****************************************************************/


		
   // hr_set_clkout_div( false, 2 );
    hr_set_int_clk_div( 0 );

    return 0;
}

uint8_t hr_set_settings( void )
{
    uint8_t reg = DIAGNOSIS;
    uint8_t temp[4] = { 0 };
		
		temp[0]  = reg;
    temp[3] |= ( 1 << DIAG_SW_RST );
    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp, 4 );
    
    return 0;
}

uint8_t hr_set_led2_sample_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = SMPL_LED2_ST;
    uint8_t reg_end = SMPL_LED2_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 ) //Checking to see if the bit count is > 16
        return -1;
		temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,  temp_st, 4 );
		temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr_set_led1_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = LED1_ST;
    uint8_t reg_end = LED1_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp_end, 4 );

    return 0;

}

uint8_t hr_set_led3_sample_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = SMPL_LED3_ST;
    uint8_t reg_end = SMPL_LED3_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,  temp_end, 4 );

    return 0;

}

uint8_t hr_set_led1_sample_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = SMPL_LED1_ST;
    uint8_t reg_end = SMPL_LED1_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,  temp_end, 4 );

    return 0;

}

uint8_t hr_set_led2_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = LED2_ST;
    uint8_t reg_end = LED2_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr_set_amb1_sample_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = SMPL_AMB1_ST;
    uint8_t reg_end = SMPL_AMB1_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;


    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,  temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr_set_led2_convert_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = LED2_CONV_ST;
    uint8_t reg_end = LED2_CONV_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr_set_led3_convert_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = LED3_CONV_ST;
    uint8_t reg_end = LED3_CONV_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;


    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp_end, 4 );

    return 0;

}

uint8_t hr_set_led1_convert_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = LED1_CONV_ST;
    uint8_t reg_end = LED1_CONV_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

		I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;


    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr_set_amb1_convert_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = AMB1_CONV_ST;
    uint8_t reg_end = AMB1_CONV_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;
		
		I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr_set_adc_reset0_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = ADC_RST_P0_ST;
    uint8_t reg_end = ADC_RST_P0_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

		I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr_set_adc_reset1_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = ADC_RST_P1_ST;
    uint8_t reg_end = ADC_RST_P1_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp_end, 4 );

    return 0;

}

uint8_t hr_set_adc_reset2_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = ADC_RST_P2_ST;
    uint8_t reg_end = ADC_RST_P2_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535  )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp_st,4);

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr_set_adc_reset3_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = ADC_RST_P3_ST;
    uint8_t reg_end = ADC_RST_P3_END;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;


    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr_set_prpct_count( uint16_t count )
{
    uint8_t reg = PRPCT;
    uint8_t temp[4] = { 0 };

    if( count > 65535 )
        return -1;
		
		temp[0] = reg;
    temp[2] = count >> 8;
    temp[3] = (uint8_t)count;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp, 4 );

    return 0;

}

//*************** The Following Sections Need Work**************

uint8_t hr_set_timer_and_average_num( bool enable, uint8_t av_num )
{
    uint8_t reg = TIM_NUMAV;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;
    if( av_num > 15 || av_num < 0 )
        return -1;
		
    if( enable )
    {
        temp[2] |= ( 1 << TIMEREN );//1 gets set Byte 2, 8 bit in reg with no shift
        temp[3] |= ( av_num << NUMAV ); //av_num gets set Byte 1, no shift needed
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp, 4 );
    }
    else
    {
        temp[3] |= ( av_num << NUMAV );
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp, 4 );
    }

    return 0;

}

uint8_t hr_set_seperate_tia_gain( bool seperate, uint8_t cf_setting,
                                   uint8_t gain_setting )
{
    uint8_t reg = TIA_GAINS2;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

    if( cf_setting > 7 || gain_setting > 7 )
        return -1;

    if( seperate )
    {
        temp[2] = TIA_ENSEPGAIN;
        temp[3] |= ( cf_setting << TIA_CF_SEP );
        temp[3] |= ( gain_setting << TIA_GAIN_SEP );
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp, 4 );
    }
    else
    {
        temp[3] |= ( cf_setting << TIA_CF_SEP );
        temp[3] |= ( gain_setting << TIA_GAIN_SEP );
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp, 4 );
    }
    
    return 0;

}

uint8_t hr_set_tia_gain( bool replace, uint8_t cf_setting,
                             uint8_t gain_setting )
{
    uint8_t reg = TIA_GAINS1;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;
    if( cf_setting > 7 || gain_setting > 7 )
        return -1;

    if( replace )
    {
        temp[2] = TIA_PROG_TG_EN;
        temp[3] |= ( cf_setting << TIA_CF );
        temp[3] |= ( gain_setting << TIA_GAIN );
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp, 4 );
    }
    else
    {
        temp[2] = 0;
        temp[3] |= ( cf_setting << TIA_CF_SEP );
        temp[3] |= ( gain_setting << TIA_GAIN_SEP );
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp, 4);
    }

    return 0;

}

uint8_t hr_replace_adc( bool replace )
{
    uint8_t reg = TIA_GAINS1;
    uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;
		//enable reading of regs
    hr_read_enable();
    I2CRead(I2C2_BASE,I2C_SLAVE_ADDRESS,  reg, temp_read, 3 );
    hr_read_disable();

    if( replace )
    {
        temp_write[2] |= TIA_PROG_TG_EN;
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_write, 4 );
    }
    else
    {
        temp_write[2] &= ~( TIA_PROG_TG_EN );
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_write, 4 );
    }

    return 0;

}

uint8_t hr_set_led_currents( uint8_t led1_current, uint8_t led2_current,
                              uint8_t led3_current )
{
    uint8_t reg = LED_CONFIG;
    uint8_t temp[4] = { 0 };
    unsigned long currents = 0;
		temp[0] = reg;

    if( led1_current > 63 ||
        led2_current > 63 ||
        led3_current > 63 )
        return -1;

    currents |= ( led3_current << ILED3 );
    currents |= ( led2_current << ILED2 );
    currents |= led1_current << ILED1;

    temp[3] |= currents;
    temp[2] |= currents >> 8;
    temp[1] |= currents >> 16;


    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp, 4 );

    return 0;

}

uint8_t hr_set_dynamic_settings( uint8_t Dyn1, uint8_t ILED_2X,uint8_t Dyn2,uint8_t OSC_ENABLE,uint8_t Dyn3,
uint8_t Dyn4,uint8_t PDNRX,uint8_t PDNAFE )
{
    uint8_t reg = SETTINGS;
			unsigned long buffer = 0;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

    buffer |= ( Dyn1 << STT_DYNMC1 );
    buffer |= ( ILED_2X << STT_ILED_2X );
    buffer |= ( Dyn2 << STT_DYNMC2 );
    buffer |= ( OSC_ENABLE<< STT_OSC_EN );
    buffer |= ( Dyn3 << STT_DYNMC3 );
    buffer |= ( Dyn4 << STT_DYNMC4 );
    buffer |= ( PDNRX << STT_PDNRX );
    buffer |= ( PDNAFE << STT_PDNAFE );

    temp[3] |= buffer;
    temp[2] |= buffer >> 8;
    temp[1] |= buffer >> 16;


    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp, 4 );

    return 0;
}
//Only used for internal clock mode
uint8_t hr_set_clkout_div( bool enable, uint8_t div )
{
    uint8_t reg = CLKOUT;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

    if( div > 15 )
        return -1;

    if( enable )
    {
        temp[2] = ( 1 << CLKOUT_EN );
        temp[3] = ( div << CLKOUT_DIV );
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp, 4 );
    }
    else
    {
        temp[3] = ( div << CLKOUT_DIV );
        I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp, 4 );
    }

    return 0;

}

uint32_t hr_get_led1_val( void )
{
    uint8_t reg = LED1VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead(I2C2_BASE,I2C_SLAVE_ADDRESS,  reg, temp, 3 );

    retval |= (uint32_t)temp[0] << 16;
    retval |= (uint32_t)temp[1] << 8;
    retval |= (uint32_t)temp[2];
//		if (retval>>23==1) // check if the ADC value is positive or negative
//{

//retval &= 0x003FFFFF; // convert it to a 22 bit value
//return (retval^0xFFC00000);
//}

    return retval;

}

uint32_t hr_get_led2_val( void )
{
    uint8_t reg = LED2VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead(I2C2_BASE,I2C_SLAVE_ADDRESS,  reg, temp, 3 );
    
    retval |= (uint32_t)temp[0] << 16;
    retval |= (uint32_t)temp[1] << 8;
    retval |= (uint32_t)temp[2];
	//	if (retval & 0x00200000) // check if the ADC value is positive or negative
//{
//retval &= 0x003FFFFF; // convert it to a 22 bit value
//return (retval^0xFFC00000);
//}

    return retval;

}

uint32_t hr_get_led3_val( void )
{
    uint8_t reg = LED3VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead( I2C2_BASE,I2C_SLAVE_ADDRESS, reg, temp, 3 );

    retval |= (uint32_t)temp[0] << 16;
    retval |= (uint32_t)temp[1] << 8;
    retval |= (uint32_t)temp[2];
	//	if (retval & 0x00200000) // check if the ADC value is positive or negative
//{
//retval &= 0x003FFFFF; // convert it to a 22 bit value
//return (retval^0xFFC00000);
//}

    return retval;

}

uint32_t hr_get_amb1_val( void )
{
    uint8_t reg = ALED1VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead(I2C2_BASE, I2C_SLAVE_ADDRESS, reg, temp, 3 );

    retval |= ( temp[0] << 16 );
    retval |= ( temp[1] << 8 );
    retval |= temp[2];
		
	//	if (retval & 0x00200000) // check if the ADC value is positive or negative
//{
//retval &= 0x003FFFFF; // convert it to a 22 bit value
//return (retval^0xFFC00000);
//}

    return retval;

}

uint32_t hr_get_led2_amb2_val( void )
{
    uint8_t reg = LED2_ALED2VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead(I2C2_BASE,I2C_SLAVE_ADDRESS,  reg, temp, 3 );

    retval |= ( temp[0] << 16 );
    retval |= ( temp[1] << 8 );
    retval |= temp[2];
		
//		if (retval & 0x00200000) // check if the ADC value is positive or negative
//{
//retval &= 0x003FFFFF; // convert it to a 22 bit value
//return (retval^0xFFC00000);
//}

    return retval;

}

uint32_t hr_get_led1_amb1_val( void )
{
    uint8_t reg = LED1_ALED1VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    hr_read_enable();
    I2CRead(I2C2_BASE,I2C_SLAVE_ADDRESS,  reg, temp, 3 );
    hr_read_disable();
    
    retval |= ( temp[0] << 16 );
    retval |= ( temp[1] << 8 );
    retval |= temp[2];

    return retval;

}

uint8_t hr_is_pd_shorted( void )
{
    uint8_t reg = PD_SHORT_FLAG;
    uint8_t temp[3] = { 0 };

    I2CRead(I2C2_BASE,I2C_SLAVE_ADDRESS,  reg, temp, 3 );

    if( temp[2] == 1 )
        return -1;
    else
        return 0;

}

//uint8_t hr_set_inp_inn_settings( inm_inn_t* inp_inn_setting )
uint8_t hr_set_inp_inn_settings(uint8_t CLKDIV_EXT_SET)
{
    uint8_t reg = PD_INP_EXT;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

   // if( inp_inn_setting->ext_div > 7 )
   //     return -1;

    //temp[2] = //( inp_inn_setting->pd_setting << PD_DISCONNECT );
    //temp[3] = //( inp_inn_setting->short_setting << ENABLE_INPUT_SHORT );
    //temp[3] = 0x05;//inp_inn_setting->ext_div;
		temp[3] =	CLKDIV_EXT_SET << CLKDIV_EXTMODE;
    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp, 4 );

    return 0;

}

uint8_t hr_set_pdn_cycle_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = PDNCYCLESTC;
    uint8_t reg_end = PDNCYCLEENDC;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,  temp_end, 4 );

    return 0;


}

uint8_t hr_set_prgrmmbl_timing_start_end( uint16_t start, uint16_t end )
{
    uint8_t reg_st = PROG_TG_STC;
    uint8_t reg_end = PROG_TG_ENDC;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;
		
I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;


}

uint8_t hr_set_led3_start_stop( uint16_t start, uint16_t end )
{
    uint8_t reg_st = LED3LEDSTC;
    uint8_t reg_end = LED3LEDENDC;
    uint8_t temp_st[4] = { 0 };
    uint8_t temp_end[4] = { 0 };

    if( start > 65535 || end > 65535 )
        return -1;

    temp_st[0] = reg_st;
    temp_st[2] = start >> 8;
    temp_st[3] = (uint8_t)start;

I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_st, 4 );

   temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;


}

uint8_t hr_set_int_clk_div( uint8_t div )
{
    uint8_t reg = CLKDIV_PRF;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;
		
    if( div > 7 )
        return -1;

    temp[3] = div;
    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp, 4 );

    return 0;

}

uint8_t hr_set_offdac_settings( uint8_t offdac_pol_led2,uint8_t offdac_set_led2,uint8_t offdac_pol_amb1,
uint8_t offdac_set_amb1,uint8_t offdac_pol_led1,uint8_t offdac_set_led1 ,uint8_t offdac_pol_amb2,uint8_t offdac_set_amb2)
{
    uint8_t reg = DAC_SETTING;
			unsigned long buffer = 0;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;


    buffer |= ( offdac_pol_led2 << POL_OFFDAC_LED2 );
    buffer |= ( offdac_set_led2 << I_OFFDAC_LED2 );
    buffer |= ( offdac_pol_amb1 << POL_OFFDAC_AMB1 );
    buffer |= ( offdac_set_amb1 << I_OFFDAC_AMB1 );
    buffer |= ( offdac_pol_led1 << POL_OFFDAC_LED1 );
    buffer |= ( offdac_set_led1 << I_OFFDAC_LED1 );
    buffer |= ( offdac_pol_amb2 << POL_OFFDAC_LED3 );
    buffer |= ( offdac_set_amb2 << I_OFFDAC_LED3 );

    temp[3] |= buffer;
    temp[2] |= buffer >> 8;
    temp[1] |= buffer >> 16;


    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp, 4 );

    return 0;

}


//Enables Registers to be read from setting bit in reg 0x00
uint8_t hr_read_enable( void )
{
    uint8_t reg = DIAGNOSIS;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

    temp[3] |= ( 1 << DIAG_REG_READ );

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS, temp, 4 );

    return 0;

}

uint8_t hr_read_disable( void )
{
    uint8_t reg = DIAGNOSIS;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

    temp[3] &= ~( 1 << DIAG_REG_READ );

    I2Cwrite(I2C2_BASE,I2C_SLAVE_ADDRESS,temp, 4 );

    return 0;

}





