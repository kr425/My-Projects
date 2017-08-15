//Initialization of TI AFE4404 HR Analog Front end


#include <stdint.h>
#include "AFE4404_REG_SETUP.h"
#include "HRV_I2C_.h"
#define I2C_SLAVE_ADDRESS       0x58


uint8_t AFE4404_init( uint8_t address, dynamic_modes_t* dyn_modes )
{
    //hr3_hal_init( address );
	  SetupI2C();
	//enable software Reset and Disable counter reset and disable register readout mode
	
    hr3_set_settings( sw_reset_en,diag_mode_dis,susp_count_dis,reg_read_dis );
    hr3_set_led2_start_end( 0, 399 );
    hr3_set_led2_sample_start_end( 100, 399 );
    hr3_set_adc_reset0_start_end( 401, 407 );
    hr3_set_led2_convert_start_end( 409, 1468 );
    hr3_set_led3_start_stop( 401, 800 );
    hr3_set_led3_sample_start_end( 501, 800 );
    hr3_set_adc_reset1_start_end( 1470, 1476 );
    hr3_set_led3_convert_start_end( 1478, 2537 );
    hr3_set_led1_start_end( 802, 1201 );
    hr3_set_led1_sample_start_end( 902, 1201 );
    hr3_set_adc_reset2_start_end( 2539, 2545 );
    hr3_set_led1_convert_start_end( 2547, 3606 );
    hr3_set_amb1_sample_start_end( 1303, 1602 );
    hr3_set_adc_reset3_start_end( 3608, 3614 );
    hr3_set_amb1_convert_start_end( 3616, 4675 );
    hr3_set_pdn_cycle_start_end( 5475, 39199 );
    hr3_set_prpct_count( 39999 );//Clock timing for CLKDIV_PRF  = 1
		/*
		
		hr3_set_led2_start_end( 0, 399*1.5 );
    hr3_set_led2_sample_start_end( 80*1.5, 399*1.5 );
    hr3_set_adc_reset0_start_end( 401*1.5, 407*1.5 );
    hr3_set_led2_convert_start_end( 408*1.5, 1467*1.5 );
    hr3_set_led3_start_stop( 400*1.5, 799*1.5 );
    hr3_set_led3_sample_start_end( 480*1.5, 799*1.5 );
    hr3_set_adc_reset1_start_end( 1469*1.5, 1475*1.5 );
    hr3_set_led3_convert_start_end( 1476*1.5, 2535*1.5 );
    hr3_set_led1_start_end( 800*1.5, 1199*1.5 );
    hr3_set_led1_sample_start_end( 880*1.5, 1199*1.5 );
    hr3_set_adc_reset2_start_end( 2537*1.5, 2543*1.5 );
    hr3_set_led1_convert_start_end( 2544*1.5, 3603*1.5 );
    hr3_set_amb1_sample_start_end( 1279*1.5, 1598*1.5 );
    hr3_set_adc_reset3_start_end( 3605*1.5, 3611*1.5 );
    hr3_set_amb1_convert_start_end( 3612*1.5, 4671*1.5 );
    hr3_set_pdn_cycle_start_end( 5471*1.5, 39199*1.5 );
    hr3_set_prpct_count( 39999*1.5 );//Clock timing for CLKDIV_PRF  = 1
*/

    //Clock Timing for CLKDIV_PRF = 5
   /* hr3_set_led2_start_end( 0, 79 );
    hr3_set_led2_sample_start_end( 16, 79 );
    hr3_set_adc_reset0_start_end( 80, 81 );
    hr3_set_led2_convert_start_end( 81, 293 );
    hr3_set_led3_start_stop( 80, 159 );
    hr3_set_led3_sample_start_end( 96, 159 );
    hr3_set_adc_reset1_start_end( 293, 295 );
    hr3_set_led3_convert_start_end( 295, 507 );
    hr3_set_led1_start_end( 160, 239 );
    hr3_set_led1_sample_start_end( 176, 239 );
    hr3_set_adc_reset2_start_end( 507, 508 );
    hr3_set_led1_convert_start_end( 508, 720 );
    hr3_set_amb1_sample_start_end( 255, 319 );
    hr3_set_adc_reset3_start_end( 721, 722 );
    hr3_set_amb1_convert_start_end( 722, 934 );
    hr3_set_pdn_cycle_start_end( 1094, 7839 );
    hr3_set_prpct_count( 7999 );        */
    
    hr3_set_timer_and_average_num( true, 3 );
    hr3_set_seperate_tia_gain( true, 0, 4 );
    hr3_set_tia_gain( false, 0, 3 );
    hr3_set_led_currents( 63,63,63 );
		//hr3_replace_adc( true );
		
		//division ratio for clk_out
		hr3_set_inp_inn_settings();
    hr3_set_dynamic_settings( dyn_modes );
   // hr3_set_clkout_div( false, 2 );
    hr3_set_int_clk_div( 0 );

    return 0;
}

uint8_t hr3_set_settings( sw_reset_t sw_reset, diag_mode_t diag_mode,
                          susp_count_t susp_counter, reg_read_t reg_read )
{
    uint8_t reg = DIAGNOSIS;
    uint8_t temp[4] = { 0 };
		
		temp[0]  = reg;
    temp[3] |= ( sw_reset << DIAG_SW_RST );
    temp[3] |= ( diag_mode << DIAG_EN );
    temp[3] |= ( susp_counter << DIAG_TM_CNT_RST );
    temp[3] |= ( reg_read << DIAG_REG_READ );

    I2Cwrite(I2C_SLAVE_ADDRESS, temp, 4 );
    
    return 0;
}

uint8_t hr3_set_led2_sample_start_end( uint16_t start, uint16_t end )
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

    I2Cwrite(I2C_SLAVE_ADDRESS,  temp_st, 4 );
		temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr3_set_led1_start_end( uint16_t start, uint16_t end )
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

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS,temp_end, 4 );

    return 0;

}

uint8_t hr3_set_led3_sample_start_end( uint16_t start, uint16_t end )
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

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS,  temp_end, 4 );

    return 0;

}

uint8_t hr3_set_led1_sample_start_end( uint16_t start, uint16_t end )
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

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS,  temp_end, 4 );

    return 0;

}

uint8_t hr3_set_led2_start_end( uint16_t start, uint16_t end )
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

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr3_set_amb1_sample_start_end( uint16_t start, uint16_t end )
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


    I2Cwrite(I2C_SLAVE_ADDRESS,  temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr3_set_led2_convert_start_end( uint16_t start, uint16_t end )
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

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr3_set_led3_convert_start_end( uint16_t start, uint16_t end )
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

I2Cwrite(I2C_SLAVE_ADDRESS,temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;


    I2Cwrite(I2C_SLAVE_ADDRESS,temp_end, 4 );

    return 0;

}

uint8_t hr3_set_led1_convert_start_end( uint16_t start, uint16_t end )
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

		I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;


    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr3_set_amb1_convert_start_end( uint16_t start, uint16_t end )
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
		
		I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr3_set_adc_reset0_start_end( uint16_t start, uint16_t end )
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

		I2Cwrite(I2C_SLAVE_ADDRESS,temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr3_set_adc_reset1_start_end( uint16_t start, uint16_t end )
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

I2Cwrite(I2C_SLAVE_ADDRESS,temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS,temp_end, 4 );

    return 0;

}

uint8_t hr3_set_adc_reset2_start_end( uint16_t start, uint16_t end )
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

I2Cwrite(I2C_SLAVE_ADDRESS,temp_st,4);

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr3_set_adc_reset3_start_end( uint16_t start, uint16_t end )
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

I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;


    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;

}

uint8_t hr3_set_prpct_count( uint16_t count )
{
    uint8_t reg = PRPCT;
    uint8_t temp[4] = { 0 };

    if( count > 65535 )
        return -1;
		
		temp[0] = reg;
    temp[2] = count >> 8;
    temp[3] = (uint8_t)count;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp, 4 );

    return 0;

}

//*************** The Following Sections Need Work**************

uint8_t hr3_set_timer_and_average_num( bool enable, uint8_t av_num )
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
        I2Cwrite(I2C_SLAVE_ADDRESS, temp, 4 );
    }
    else
    {
        temp[3] |= ( av_num << NUMAV );
        I2Cwrite(I2C_SLAVE_ADDRESS,temp, 4 );
    }

    return 0;

}

uint8_t hr3_set_seperate_tia_gain( bool seperate, uint8_t cf_setting,
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
        I2Cwrite(I2C_SLAVE_ADDRESS, temp, 4 );
    }
    else
    {
        temp[3] |= ( cf_setting << TIA_CF_SEP );
        temp[3] |= ( gain_setting << TIA_GAIN_SEP );
        I2Cwrite(I2C_SLAVE_ADDRESS, temp, 4 );
    }
    
    return 0;

}

uint8_t hr3_set_tia_gain( bool replace, uint8_t cf_setting,
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
        I2Cwrite(I2C_SLAVE_ADDRESS,temp, 4 );
    }
    else
    {
        temp[2] = 0;
        temp[3] |= ( cf_setting << TIA_CF_SEP );
        temp[3] |= ( gain_setting << TIA_GAIN_SEP );
        I2Cwrite(I2C_SLAVE_ADDRESS,temp, 4);
    }

    return 0;

}

uint8_t hr3_replace_adc( bool replace )
{
    uint8_t reg = TIA_GAINS1;
    uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;
		//enable reading of regs
    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp_read, 3 );
    hr3_read_disable();

    if( replace )
    {
        temp_write[2] |= TIA_PROG_TG_EN;
        I2Cwrite(I2C_SLAVE_ADDRESS, temp_write, 4 );
    }
    else
    {
        temp_write[2] &= ~( TIA_PROG_TG_EN );
        I2Cwrite(I2C_SLAVE_ADDRESS, temp_write, 4 );
    }

    return 0;

}

uint8_t hr3_set_led_currents( uint8_t led1_current, uint8_t led2_current,
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

    currents |= ( led3_current << 12 );
    currents |= ( led2_current << 6 );
    currents |= led1_current;

    temp[3] |= currents;
    temp[2] |= currents >> 8;
    temp[1] |= currents >> 16;


    I2Cwrite(I2C_SLAVE_ADDRESS, temp, 4 );

    return 0;

}

uint8_t hr3_set_dynamic_settings( dynamic_modes_t* modes )
{
    uint8_t reg = SETTINGS;
			unsigned long buffer = 0;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

    buffer |= ( modes->transmit << STT_DYNMC1 );
    buffer |= ( modes->curr_range << STT_ILED_2X );
    buffer |= ( modes->adc_power << STT_DYNMC2 );
    buffer |= ( modes->clk_mode << STT_OSC_EN );
    buffer |= ( modes->tia_power << STT_DYNMC3 );
    buffer |= ( modes->rest_of_adc << STT_DYNMC4 );
    buffer |= ( modes->afe_rx_mode << STT_PDNRX );
    buffer |= ( modes->afe_mode << STT_PDNAFE );

    temp[3] |= buffer;
    temp[2] |= buffer >> 8;
    temp[1] |= buffer >> 16;


    I2Cwrite(I2C_SLAVE_ADDRESS,temp, 4 );

    return 0;
}
uint8_t hr3_set_transmit_enable( transmitter_t transmit )
{
    uint8_t reg = SETTINGS;
			unsigned long buffer = 0;
		uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;
  

    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS, reg, temp_read, 3 );
    hr3_read_disable();

    buffer |= ( transmit << STT_DYNMC1 );
    temp_write[3] |= buffer;
    temp_write[2] |= buffer >> 8;
    temp_write[1] |= buffer >> 16;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_write, 4 );

    return 0;

}

uint8_t hr3_set_current_range( led_current_range_t curr_range )
{
    uint8_t reg = SETTINGS;
		unsigned long buffer = 0;
		uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;
  

    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp_read, 3 );
    hr3_read_disable();

    buffer |= ( curr_range << STT_ILED_2X );
    temp_write[3] |= buffer;
    temp_write[2] |= buffer >> 8;
    temp_write[1] |= buffer >> 16;

    I2Cwrite(I2C_SLAVE_ADDRESS,temp_write, 4 );

    return 0;

}

uint8_t hr3_set_adc_power( adc_pwer_t adc_power )
{
    uint8_t reg = SETTINGS;
    unsigned long buffer = 0;
		uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;

    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp_read, 3 );
    hr3_read_disable();

    buffer |= ( adc_power << STT_DYNMC2 );
    temp_write[3] |= buffer;
    temp_write[2] |= buffer >> 8;
    temp_write[1] |= buffer >> 16;

    I2Cwrite(I2C_SLAVE_ADDRESS,temp_write, 4 );

    return 0;

}

uint8_t hr3_set_clock_mode( clk_mode_t clk_mode )
{
    uint8_t reg = SETTINGS;
    unsigned long buffer = 0;
		uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;

    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp_read, 3 );
    hr3_read_disable();

    buffer |= ( clk_mode << STT_OSC_EN );
    temp_write[3] |= buffer;
    temp_write[2] |= buffer >> 8;
    temp_write[1] |= buffer >> 16;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_write, 4 );

    return 0;

}

uint8_t hr3_set_tia_power( tia_pwer_t tia_power )
{
		
    uint8_t reg = SETTINGS;
		unsigned long buffer = 0;
    uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;
    

    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp_read, 3 );
    hr3_read_disable();

    buffer |= ( tia_power << STT_DYNMC3 );
    temp_write[3] |= buffer;
    temp_write[2] |= buffer >> 8;
    temp_write[1] |= buffer >> 16;

    I2Cwrite(I2C_SLAVE_ADDRESS,temp_write, 4 );

    return 0;

}

uint8_t hr3_set_rest_of_adc( rest_of_adc_t rest_of_adc )
{
    uint8_t reg = SETTINGS;
			unsigned long buffer = 0;
    uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;


    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp_read, 3 );
    hr3_read_disable();

    buffer |= ( rest_of_adc << STT_DYNMC4 );
    temp_write[3] |= buffer;
    temp_write[2] |= buffer >> 8;
    temp_write[1] |= buffer >> 16;

    I2Cwrite(I2C_SLAVE_ADDRESS,temp_write, 4 );

    return 0;

}

uint8_t hr3_set_afe_receive( afe_rx_mode_t afe_rx_mode )
{
    uint8_t reg = SETTINGS;
			unsigned long buffer = 0;
    uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;
  

    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp_read, 3 );
    hr3_read_disable();

    buffer |= ( afe_rx_mode << STT_PDNRX );
    temp_write[3] |= buffer;
    temp_write[2] |= buffer >> 8;
    temp_write[1] |= buffer >> 16;

    I2Cwrite(I2C_SLAVE_ADDRESS,temp_write, 4 );

    return 0;

}

uint8_t hr3_set_afe_mode( afe_mode_t afe_mode )
{
    uint8_t reg = SETTINGS;
			unsigned long buffer = 0;
    uint8_t temp_read[3] = {0};
		uint8_t temp_write[4] = {0};
		temp_write[0] = reg;
    

    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp_read, 3 );
    hr3_read_disable();

    buffer |= ( afe_mode << STT_PDNAFE );
    temp_write[3] |= buffer;
    temp_write[2] |= buffer >> 8;
    temp_write[1] |= buffer >> 16;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_write, 4 );

    return 0;

}

uint8_t hr3_set_clkout_div( bool enable, uint8_t div )
{
    uint8_t reg = CLKOUT;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

    if( div > 15 )
        return -1;

    if( enable )
    {
        temp[2] = ( 0 << CLKOUT_EN );
        temp[3] = ( div << CLKOUT_DIV );
        I2Cwrite(I2C_SLAVE_ADDRESS, temp, 4 );
    }
    else
    {
        temp[3] = ( div << CLKOUT_DIV );
        I2Cwrite(I2C_SLAVE_ADDRESS,temp, 4 );
    }

    return 0;

}

uint32_t hr3_get_led1_val( void )
{
    uint8_t reg = LED1VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp, 3 );

    retval |= (uint32_t)temp[0] << 16;
    retval |= (uint32_t)temp[1] << 8;
    retval |= (uint32_t)temp[2];

    return retval;

}

uint32_t hr3_get_led2_val( void )
{
    uint8_t reg = LED2VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp, 3 );
    
    retval |= (uint32_t)temp[0] << 16;
    retval |= (uint32_t)temp[1] << 8;
    retval |= (uint32_t)temp[2];

    return retval;

}

uint32_t hr3_get_led3_val( void )
{
    uint8_t reg = LED3VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead( I2C_SLAVE_ADDRESS, reg, temp, 3 );

    retval |= (uint32_t)temp[0] << 16;
    retval |= (uint32_t)temp[1] << 8;
    retval |= (uint32_t)temp[2];

    return retval;

}

uint32_t hr3_get_amb1_val( void )
{
    uint8_t reg = ALED1VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead( I2C_SLAVE_ADDRESS, reg, temp, 3 );

    retval |= ( temp[0] << 16 );
    retval |= ( temp[1] << 8 );
    retval |= temp[2];

    return retval;

}

uint32_t hr3_get_led2_amb2_val( void )
{
    uint8_t reg = LED2_ALED2VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp, 3 );

    retval |= ( temp[0] << 16 );
    retval |= ( temp[1] << 8 );
    retval |= temp[2];

    return retval;

}

uint32_t hr3_get_led1_amb1_val( void )
{
    uint8_t reg = LED1_ALED1VAL;
    uint8_t temp[3] = { 0 };
    uint32_t retval = 0;

    hr3_read_enable();
    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp, 3 );
    hr3_read_disable();
    
    retval |= ( temp[0] << 16 );
    retval |= ( temp[1] << 8 );
    retval |= temp[2];

    return retval;

}

uint8_t hr3_is_pd_shorted( void )
{
    uint8_t reg = PD_SHORT_FLAG;
    uint8_t temp[3] = { 0 };

    I2CRead(I2C_SLAVE_ADDRESS,  reg, temp, 3 );

    if( temp[2] == 1 )
        return -1;
    else
        return 0;

}

//uint8_t hr3_set_inp_inn_settings( inm_inn_t* inp_inn_setting )
uint8_t hr3_set_inp_inn_settings(void)
{
    uint8_t reg = PD_INP_EXT;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

   // if( inp_inn_setting->ext_div > 7 )
   //     return -1;

    //temp[2] = //( inp_inn_setting->pd_setting << PD_DISCONNECT );
    //temp[3] = //( inp_inn_setting->short_setting << ENABLE_INPUT_SHORT );
    temp[3] = 0x05;//inp_inn_setting->ext_div;
    I2Cwrite(I2C_SLAVE_ADDRESS,temp, 4 );

    return 0;

}

uint8_t hr3_set_pdn_cycle_start_end( uint16_t start, uint16_t end )
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

I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS,  temp_end, 4 );

    return 0;


}

uint8_t hr3_set_prgrmmbl_timing_start_end( uint16_t start, uint16_t end )
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
		
I2Cwrite(I2C_SLAVE_ADDRESS,temp_st, 4 );

    temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;


}

uint8_t hr3_set_led3_start_stop( uint16_t start, uint16_t end )
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

I2Cwrite(I2C_SLAVE_ADDRESS, temp_st, 4 );

   temp_end[0] = reg_end;
    temp_end[2] = end >> 8;
    temp_end[3] = (uint8_t)end;

    I2Cwrite(I2C_SLAVE_ADDRESS, temp_end, 4 );

    return 0;


}

uint8_t hr3_set_int_clk_div( uint8_t div )
{
    uint8_t reg = CLKDIV_PRF;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;
		
    if( div > 7 )
        return -1;

    temp[3] = div;
    I2Cwrite(I2C_SLAVE_ADDRESS, temp, 4 );

    return 0;

}

uint8_t hr3_set_offdac_settings( offdac_settings_t* settings )
{
    uint8_t reg = DAC_SETTING;
			unsigned long buffer = 0;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;


    buffer |= ( settings->offdac_pol_led2 << POL_OFFDAC_LED2 );
    buffer |= ( settings->offdac_set_led2 << I_OFFDAC_LED2 );
    buffer |= ( settings->offdac_pol_amb1 << POL_OFFDAC_AMB1 );
    buffer |= ( settings->offdac_set_amb1 << I_OFFDAC_AMB1 );
    buffer |= ( settings->offdac_pol_led1 << POL_OFFDAC_LED1 );
    buffer |= ( settings->offdac_set_led1 << I_OFFDAC_LED1 );
    buffer |= ( settings->offdac_pol_amb2 << POL_OFFDAC_LED3 );
    buffer |= ( settings->offdac_set_amb2 << I_OFFDAC_LED3 );

    temp[3] |= buffer;
    temp[2] |= buffer >> 8;
    temp[1] |= buffer >> 16;


    I2Cwrite(I2C_SLAVE_ADDRESS,temp, 4 );

    return 0;

}


//Enables Registers to be read from setting bit in reg 0x00
uint8_t hr3_read_enable( void )
{
    uint8_t reg = DIAGNOSIS;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

    temp[3] |= ( 1 << DIAG_REG_READ );

    I2Cwrite(I2C_SLAVE_ADDRESS, temp, 4 );

    return 0;

}

uint8_t hr3_read_disable( void )
{
    uint8_t reg = DIAGNOSIS;
    uint8_t temp[4] = { 0 };
		temp[0] = reg;

    temp[3] &= ~( 1 << DIAG_REG_READ );

    I2Cwrite(I2C_SLAVE_ADDRESS,temp, 4 );

    return 0;

}




