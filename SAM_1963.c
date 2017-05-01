/*
 * SAM_1963.c
 *
 * Created: 4/29/2017 11:26:43 AM
 *  Author: Chris Dickinson
 */ 

#include "SAM_1963.h"

#pragma GCC optimize 0

//--------------	FUNCTION DEFINITION		--------------------
void tft_delay(uint32_t time)
{
	//Crude delay to ensure reference clock can catch up with host MCU clock
	volatile uint32_t count;
	for(count = 0; count < (time * NS_DELAY); count++);
}

void tft_write(uint32_t data, bool command)
{
	// Writes data to the TFT using the 8080 timing and bit-banging 

	if (command)
	{
		set_mode_cmd();		// COMMAND | D/C# -> LO
	}
	else
	{
		set_mode_data();	// COMMAND | D/C# -> HI	
	}
	
	write_data_port(data);				
	set_write_enable();	// WR# -> LO, RD# -> HI, DDR -> OUTPUT
	tft_delay(2);	
	tft_select( 0 );	//Selects the TFT
	tft_delay(5);
	tft_select( 1 );	//Latches the data
	tft_delay(2);	
}

uint32_t tft_read( )
{
	uint32_t data;
	
	// Reads data from the TFT using the 8080 timing and bit-banging 
	set_read_enable();	// WR# -> HI, RD# -> LO, DDR -> INPUT
	tft_delay(2);
	tft_select( 0 );	//Selects the TFT
	tft_delay(5);
	data = read_data_port();
	tft_select( 1 );	//Latches the data
	tft_delay(2);
	return data;
}

void tft_init( void )
{
	//Initializes TFT display/controller
	data_port_init();
	ctl_bits_init();
}

void data_port_init( void )
{
	//Initializes selected port pins in output mode, sets pins to low
	ioport_set_port_dir(TFT_LOW_PORT, TFT_LOW_MASK, IOPORT_DIR_OUTPUT);
	ioport_set_port_dir(TFT_HIGH_PORT, TFT_HIGH_MASK, IOPORT_DIR_OUTPUT);
	
	write_data_port( (uint32_t) 0x0 );
}

void ctl_bits_init( void )
{
	//Initializes selected control pins in output mode, and sets pins to default value
	ioport_set_pin_dir(BL_EN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(DC_SEL, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(WR_8080, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(RD_8080, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(CS_BIT, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(TFT_ON, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(TFT_RST, IOPORT_DIR_OUTPUT);
	
	//Enable backlight
	bl_enable(true);
	
	//Set command mode
	set_mode_cmd();
	//ioport_set_pin_level(DC_SEL, 0);
	
	//Disable channel select
	tft_select(1);
	//ioport_set_pin_level(CS_BIT, 1);
	
	write_data_port(0xFACE);
	
	//Go to write mode
	set_write_enable();
	//ioport_set_pin_level(WR_8080, 1);
	//ioport_set_pin_level(RD_8080, 1);
	
	#ifdef	RST_TFT_ON_INIT
		//Reset TFT controller
		rst_tft();
	#else
		ioport_set_pin_level(TFT_RST, 1);
	#endif
	
	//Turn on TFT
	tft_enable(true);
	//ioport_set_pin_level(TFT_ON, 1);
}

void write_data_port( uint32_t data_out )
{
	//Writes word to data pins
	uint32_t buffer_low;
	uint32_t buffer_high;

	buffer_low = ((data_out & 0xFF) << TFT_LOW_OFFSET);
	buffer_high = ((data_out & 0xFF00) << (TFT_HIGH_OFFSET - 8));
	
	ioport_set_port_level(TFT_LOW_PORT, TFT_LOW_MASK, buffer_low);
	ioport_set_port_level(TFT_HIGH_PORT, TFT_HIGH_MASK, buffer_high);
}

uint32_t read_data_port( )
{
	//Reads word from data pins
	uint32_t buffer_low;
	uint32_t buffer_high;
	uint32_t data_in;

	buffer_low = ioport_get_port_level(TFT_LOW_PORT, TFT_LOW_MASK);
	buffer_high = ioport_get_port_level(TFT_HIGH_PORT, TFT_HIGH_MASK);

	data_in = ((buffer_high & TFT_HIGH_MASK) >> (TFT_HIGH_OFFSET - 8)) | ((buffer_low & TFT_LOW_MASK) >> (TFT_LOW_OFFSET));

	return data_in;
}

//void set_mode_cmd( void )
//{
	////Sets command mode
	//ioport_set_pin_level(DC_SEL, 0);
//}

//void set_mode_data( void )
//{
	////Sets data mode
	//ioport_set_pin_level(DC_SEL, 1);
//}

void set_write_enable()
{
	//Enables write functionality to the display, opposite of read_enable()
	ioport_set_pin_level(RD_8080, 1);	
	ioport_set_pin_level(WR_8080, 0);

	//Set port as output
	ioport_set_port_dir(TFT_LOW_PORT, TFT_LOW_MASK, IOPORT_DIR_OUTPUT);
	ioport_set_port_dir(TFT_HIGH_PORT, TFT_HIGH_MASK, IOPORT_DIR_OUTPUT);
}

void set_read_enable()
{
	//Set port as input
	ioport_set_port_dir(TFT_LOW_PORT, TFT_LOW_MASK, IOPORT_DIR_INPUT);
	ioport_set_port_dir(TFT_HIGH_PORT, TFT_HIGH_MASK, IOPORT_DIR_INPUT);	
	
	//Enables read functionality from the display, opposite of write_enable()
	ioport_set_pin_level(WR_8080, 1);
	ioport_set_pin_level(RD_8080, 0);
}

//void tft_select( bool cs_state )
//{
	////Selects or deselects the display based on cs_state (8080 Mode Timing Diagram)
	//ioport_set_pin_level(BL_EN, cs_state);
//}

//void bl_enable(	bool bl_state )
//{
	////Enables or disables backlight based on bl_state
	//ioport_set_pin_level(BL_EN, bl_state);
//}

//void tft_enable( bool en_state )
//{
	////Turns TFT on or off based on en_state
	//ioport_set_pin_level(TFT_ON, en_state);
//}

void rst_tft( void )
{
	//Reset TFT controller
	ioport_set_pin_level(TFT_RST, 0);
	
	//Crude delay to ensure reference clock can catch up with host MCU clock
	tft_delay(1000);
	
	ioport_set_pin_level(TFT_RST, 1);
}

