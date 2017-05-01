/*
 * SSD1963_CommandLib.c
 *
 * Created: 4/29/2017 11:34:19 AM
 *  Author: Chris Dickinson
 */ 

#include "SSD1963_CommandLib.h"

#pragma GCC optimize 0

//--------------	FUNCTION DEFINITION		--------------------
void tft_nop()					// 9.1 No operation
{
	// No operation.
	// - CAN TERMINATE
	//		- write_memory_continue()
	//		- read_memory_continue()
	tft_write((uint32_t) 0x00, true);
}

void tft_soft_reset()			// 9.2 Software Reset
{
	// Software reset of the configuration register.
	// - Note:
	//		- host processor must wait [5ms] before sending any new commands
	//		- host processor must wait [120ms] before calling exit_sleep_mode()
	tft_write((uint32_t) 0x01, true);
}

uint32_t tft_get_power_mode()	// 9.3 Get Power Mode
{
	// Get the current power mode.
	
	uint32_t data;
	tft_write((uint32_t) 0x0A, true);
	tft_delay(6);
	data = tft_read();

	// Return value
	// - command_id[6]: Idle mode on/off (POR = 0)
	//		< 0: OFF , 1: ON >
	// - command_id[5]: Partial mode on/off (POR = 0)
	//		< 0: OFF , 1: ON >
	// - command_id[4]: Sleep mode on/off (POR = 0)
	//		< 0: ON , 1: OFF >
	// - command_id[3]: Display normal mode on/off (POR = 1)
	//		< 0: OFF , 1: ON >
	// - command_id[2]: Display on/off (POR = 0)
	//		< 0: OFF , 1: ON >

	return data;
}

uint32_t tft_get_address_mode()	// 9.4 Get Address Mode
{
	// Get the frame buffer to the display panel read order.
	
	uint32_t data;
	tft_write((uint32_t) 0x0B, true);
	tft_delay(6);
	data = tft_read();

	// Return value
	// - command_id[7]: Page address order (POR = 0)
	//		< 0: Top to Bottom , 1: Bottom to Top >
	// - command_id[6]: Column address order (POR = 0)
	//		< 0: Left to Right , 1: Right to Left >
	// - command_id[5]: Page / Column order (POR = 0)
	//		< 0: Normal mode , 1: Reverse mode >
	// - command_id[4]: Line address order (POR = 0)
	//	  {LCD refresh} < 0: Top to Bottom , 1: Bottom to Top >
	// - command_id[3]: RGB / BGR order (POR = 1)
	//		< 0: RGB , 1: BGR >
	// - command_id[2]: Display data latch data (POR = 0)
	//	  {LCD refresh} < 0: Left to Right , 1: Right to Left >

	return data;
}

uint32_t tft_get_pixel_format()	// 9.5 Get Pixel Format
{
	// Get the current pixel format for the RGB image data.
	
	uint32_t data;	
	tft_write((uint32_t) 0x0C, true);
	tft_delay(6);
	data = tft_read();

	// Return value
	// - command_id[6:4]: Display pixel format (POR)
	//		- 000: Reserved
	//		- 001: 3-bit/pixel
	//		- 010: 8-bit/pixel
	//		- 011: 12-bit/pixel
	//		- 100: reserved
	//		- 101: 16-bit/pixel
	//		- 110: 18-bit/pixel
	//		- 111: 24-bit/pixel
	
	return data;
}

uint32_t tft_get_display_mode()	// 9.6 Get Display mode
{
	// The display module returns the Display Image Mode status.
	
	uint32_t data;	
	tft_write((uint32_t) 0x0D, true);
	tft_delay(6);
	data = tft_read();

	// Return value
	// - command_id[7] : Vertical scrolling status (POR = 0)
	//		< 0: off , 1: on >
	// command_id[5] : Inversion on/off (POR = 0)
	//		< 0: off , 1: on >
	// command_id[2:0] : Gamma curve selection (POR = 011)
	//		- 000: Gamma curve 0
	//		- 001: Gamma curve 1
	//		- 010: Gamma curve 2
	//		- 011: Gamma curve 3
	//		- 100: Reserved
	//		- 101: Reserved
	//		- 110: Reserved
	//		- 111: Reserved

	return data;
}

uint32_t tft_get_signal_mode()	// 9.7 Get Signal Mode
{
	// Get the current display signal mode from the peripheral.
	
	uint32_t data;
	tft_write((uint32_t) 0x0E, true);
	tft_delay(6);
	data = tft_read();

	// Return value
	// - command_id[7]: Tearing effect line mode (POR = 0)
	//		< 0: OFF , 1: ON >

	return data;
}

void tft_enter_sleep_mode()		// 9.8 Enter Sleep Mode
{
	// Turn off the panel.
	// - Causes display panel to enter sleep mode
	// - Pulls GPIO0 LOW
	//		> unless set_gpio_conf() has configured GPIO0 as
	//			- normal GPIO
	//			- LCD miscellaneous signal
	//
	// - Note:
	//		- host processor must wait [5ms] before sending any new commands
	//		- host processor must wait [120ms] before calling exit_sleep_mode()
	tft_write((uint32_t) 0x10, true);	
}

void tft_exit_sleep_mode()		// 9.9 Exit Sleep Mode
{
	// Turn on the panel.
	// - Causes display panel to enter sleep mode
	// - Pulls GPIO0 HIGH
	//		> unless set_gpio_conf() has configured GPIO0 as
	//			- normal GPIO
	//			- LCD miscellaneous signal
	//
	// - Note:
	//		- host processor must wait [5ms] before sending any new commands
	//		- host processor must wait [120ms] before calling enter_sleep_mode()
		
	tft_write((uint32_t) 0x11, true);	
	tft_delay(5000);
}

void tft_enter_partial_mode()	// 9.10 Enter Partial Mode
{
	// Display module enters the Partial Display Mode.
	// - The Partial Display Mode window is described by the
	//		- set_partial_area()
	//
	// - To leave Partial Display Mode
	//		- enter_normal_mode()
	tft_write((uint32_t) 0x12, true);	
}

void tft_enter_normal_mode()	// 9.11 Enter Normal Mode
{
	// Display module enters normal mode.
	// - Partial Display Mode and Scroll Mode are OFF
	//
	// - The whole display area is used for image display.
	tft_write((uint32_t) 0x13, true);	
}

void tft_exit_invert_mode()		// 9.12 Exit Invert Mode
{
	// Display module stops inverting the image data on the display device.
	// - The frame buffer contents remain unchanged
	tft_write((uint32_t) 0x20, true);	
}

void tft_enter_invert_mode()		// 9.13 Enter Invert Mode
{
	// Display module inverts the image data on the display device.
	// - The frame buffer contents remain unchanged
	tft_write((uint32_t) 0x21, true);
}

void tft_set_gamma_curve(uint32_t gamma_curve)			// 9.14 Set Gamma Curve
{
	// Selects the gamma curve used by the display device.
	// command_id[3:0]	|	Gamma curve selection (POR = 1000)	|	GAMAS[1]	|	GAMAS[0]
	//-----------------------------------------------------------------------------------
	//	  0000		:		No gamma curve selected			:		0		:		0
	//	  0001		:			Gamma curve 0				:		0		:		0
	//	  0010		:			Gamma curve 1				:		0		:		1
	//	  0100		:			Gamma curve 2				:		1		:		0
	//	  1000		:			Gamma curve 3				:		1		:		1
	//	 Others		:	------------------------- Reserved -----------------------------
	
	tft_write((uint32_t) 0x26, true);
	tft_write(( gamma_curve & 0xF ), false);	
}


void tft_set_display_off()	// 9.15 Set Display Off
{
	// Blanks the display device.
	//	- The frame buffer contents remain unchanged.
	
	// Send command to driver	
	tft_write((uint32_t) 0x28, true);
}

void tft_set_display_on()	// 9.16 Set Display On
{
	// Show the image on the display device
	//	- The frame buffer contents remain unchanged.
	
	// Send command to driver	
	tft_write((uint32_t) 0x29, true);	
}

void tft_set_column_address( uint32_t start_col, 
							 uint32_t end_col )	// 9.17 Set Column Address
{
	uint32_t start_col_high = (start_col & 0xFF00) >> 8; 
	uint32_t start_col_low = (start_col & 0x00FF);
	uint32_t end_col_high = (end_col & 0xFF00) >> 8;
	uint32_t end_col_low = (end_col & 0x00FF);
		
	// Set the column extent of frame buffer accessed by the host processor
	// - with these functions
	//		- write_memory_continue()
	//		- read_memory_continue()

	// Note:
	//	- start column number =< end column number
	tft_write((uint32_t) 0x2A, true);
	tft_write((uint32_t) start_col_high, false);	
	tft_write((uint32_t) start_col_low, false);		
	tft_write((uint32_t) end_col_high, false);		
	tft_write((uint32_t) end_col_low, false);	
}

void tft_set_page_address( uint32_t start_pg,
						   uint32_t end_pg )	// 9.18 Set Page Address
{
	uint32_t start_pg_high = (start_pg & 0xFF00) >> 8;
	uint32_t start_pg_low = (start_pg & 0x00FF);
	uint32_t end_pg_high = (end_pg & 0xFF00) >> 8;
	uint32_t end_pg_low = (end_pg & 0x00FF);
	
	// Set the page extent of the frame buffer accessed by the host processor
	// - with these functions
	//		- write_memory_continue()
	//		- read_memory_continue()

	// Note:
	//	- start page (row) number =< end page (row) number
	tft_write((uint32_t) 0x2B, true);
	tft_write((uint32_t) start_pg_high, false);
	tft_write((uint32_t) start_pg_low, false);
	tft_write((uint32_t) end_pg_high, false);
	tft_write((uint32_t) end_pg_low, false);
}

void tft_write_memory_start()	// 9.19 Write Memory Start
{
	// Transfer image information from the host processor interface to the SSD1963
	//	- starting at the location set by
	//		- set_col_address()
	//		- set_page_address()

	// If Set Address Mode, 0x36 A[5] = 0:
	// The column and page registers are reset to the Start Column (SC) and Start Page (SP), respectively.
	//
	// If Set Address Mode, 0x36 A[5] = 1:
	// The column and page registers are reset to the Start Column (SC) and Start Page (SP), respectively.

	// -> REFER to solomon_systech_ssd1963 datasheet for additional information
	tft_write((uint32_t) 0x2C, true);
}

void tft_read_memory_start()	// 9.20 Read Memory Start
{
	// Transfer image data from the SSD1963 to the host processor interface
	//	- starting at the location set by
	//		- set_col_address()
	//		- set_page_address()

	// If Set Address Mode, 0x36 A[5] = 0:
	// The column and page registers are reset to the Start Column (SC) and Start Page (SP), respectively.
	//
	// If Set Address Mode, 0x36 A[5] = 1:
	// The column and page registers are reset to the Start Column (SC) and Start Page (SP), respectively.

	// -> REFER to solomon_systech_ssd1963 datasheet for additional information
	tft_write((uint32_t) 0x2E, true);
}

void tft_set_partial_area( uint32_t start_disp_row,
						   uint32_t end_disp_row )		// 9.21 Set Partial Area
{
	uint32_t start_disp_row_high = (start_disp_row & 0xFF00) >> 8;
	uint32_t start_disp_row_low = (start_disp_row & 0x00FF);
	uint32_t end_disp_row_high = (end_disp_row & 0xFF00) >> 8;
	uint32_t end_disp_row_low = (end_disp_row & 0x00FF);
	
	// This command defines the Partial Display mode’s display area.
	//	- two parameters:
	//		- Start Row
	//		- End Row

	//	- these parameters refer to the Frame Buffer Line Pointer

	// Note:
	//	- neither parameter can be 0x0000 or exceed the last vertical line number.

	// If End Row > Start Row
	//		- all parts of the partial area are contiguous on the display
	// If End Row > Start Row
	//		- there is wrap-around between the top and bottom of the display
	tft_write((uint32_t) 0x30, true);
	tft_write((uint32_t) start_disp_row_high, false);
	tft_write((uint32_t) start_disp_row_low, false);
	tft_write((uint32_t) end_disp_row_high, false);
	tft_write((uint32_t) end_disp_row_low, false);
}

void tft_set_scroll_area(  uint32_t top_fixed_area,
						   uint32_t vertical_scrolling_area,
						   uint32_t bottom_fixed_area)
{
	uint32_t top_fixed_area_high = (top_fixed_area & 0xFF00) >> 8;
	uint32_t top_fixed_area_low = (top_fixed_area & 0x00FF);
	uint32_t vertical_scrolling_area_high = (vertical_scrolling_area & 0xFF00) >> 8;
	uint32_t vertical_scrolling_area_low = (vertical_scrolling_area & 0x00FF);
	uint32_t bottom_fixed_area_high = (bottom_fixed_area & 0xFF00) >> 8;
	uint32_t bottom_fixed_area_low = (bottom_fixed_area & 0x00FF);	
	
	//Defines the vertical scrolling and fixed area on display area
	// -> REFER to solomon_systech_ssd1963 datasheet for additional information
	
	tft_write((uint32_t) 0x33, true);
	tft_write((uint32_t) top_fixed_area_high, false);
	tft_write((uint32_t) top_fixed_area_low, false);
	tft_write((uint32_t) vertical_scrolling_area_high, false);
	tft_write((uint32_t) vertical_scrolling_area_low, false);	
	tft_write((uint32_t) bottom_fixed_area_high, false);
	tft_write((uint32_t) bottom_fixed_area_low, false);	
}
	
void tft_set_tear_off( )
{
	//TE signal is not sent from the display module to the host processor.
	tft_write((uint32_t) 0x34, true);
}

void tft_set_tear_on( uint32_t tearing_effect)
{
	//Tearing Effect signal is sent from the display module to the host processor at the start of VFP.
	// -> REFER to solomon_systech_ssd1963 datasheet for additional information
	
	//0 - The tearing effect output line consists of V-blanking information only.
	//1 - The tearing effect output line consists of both V-blanking and H-blanking information.
	
	tft_write((uint32_t) 0x33, true);
	tft_write((uint32_t) (tearing_effect & 0x0001), false);	
}

void tft_set_address_mode( uint8_t page_address_order,	//0 - Top to bottom, pages transferred from SP (Start Page) to EP (End Page).
														//1 - Bottom to top, pages transferred from EP (End Page) to SP (Start Page).
						   uint8_t col_address_order,	//0 - Left to right, columns transferred from SC (Start Column) to EC (End Column).
														//1 - Right to left, columns transferred from EC (End Column) to SC (Start Column).
						   uint8_t col_order,			//0 - Normal mode, 1 - Reverse mode
						   uint8_t row_address_order,	//0 - LCD refresh from top line to bottom line, 1 - LCD refresh from bottom line to top line.
						   uint8_t rgb_bgr_order,		//0 - RGB, 1 - BGR
						   uint8_t display_data,		//0 - LCD refresh from left side to right side, 1 - LCD refresh from right side to left side.
						   uint8_t flip_horizontal,		//0 - Normal, 1 - Flipped
						   uint8_t flip_vertical		//0 - Normal, 1 - Flipped
						   )		// 9.25 Set Address Mode
{
	uint32_t address_mode = ((page_address_order & 0x1) << 7) |
							((col_address_order & 0x1) << 6) |
							((col_order & 0x1) << 5) |
							((row_address_order & 0x1) << 4) |
							((rgb_bgr_order & 0x1) << 3) |
							((display_data & 0x1) << 2) |
							((flip_horizontal & 0x1) << 1) |
							(flip_vertical & 0x1);

	// Set the read order from host processor to frame buffer
	//	- by address_mode[7:5] and address_mode[3].

	// Set the read order from frame buffer to the display panel
	//	- by address_mode[2:0] and address_mode[4].

	// -> REFER to solomon_systech_ssd1963 datasheet for additional information
	tft_write((uint32_t) 0x36, true);
	tft_write((uint32_t) (address_mode & 0x000F), false);
}

void tft_set_scroll_start( uint32_t vertical_scrolling_pointer )
{
	uint32_t vertical_scrolling_pointer_high = (vertical_scrolling_pointer & 0xFF00) >> 8;
	uint32_t vertical_scrolling_pointer_low = (vertical_scrolling_pointer & 0x00FF);

	//This command sets the start of the vertical scrolling area in the frame buffer. 
	//The vertical scrolling area is fully defined when this command is used with the Set Scroll Area 0x33.
	// -> REFER to solomon_systech_ssd1963 datasheet for additional information
	
	tft_write((uint32_t) 0x37, true);
	tft_write((uint32_t) vertical_scrolling_pointer_high, false);
	tft_write((uint32_t) vertical_scrolling_pointer_low, false);
}

void tft_exit_idle_mode( )
{
	//This command causes the display module to exit Idle Mode.
	//Full color depth is used for the display panel.
	tft_write((uint32_t) 0x38, true);
}

void tft_enter_idle_mode( )
{
	//This command causes the display module to enter Idle Mode.
	//In Idle Mode, color depth is reduced. 
	//Colors are shown on the display panel using the MSB of each of the R, G and B color components in the frame buffer.
	tft_write((uint32_t) 0x39, true);	
}

void tft_set_pixel_format( uint32_t pixel_format )				// 9.29 Set Pixel Format
{
	// Set the current pixel format for RGB image data
	//	pixel_format[6:4] : Display pixel format (POR = 000)
	//		- 000: Reserved
	//		- 001: 3-bit/pixel
	//		- 010: 8-bit/pixel
	//		- 011: 12-bit/pixel
	//		- 100: Reserved
	//		- 101: 16-bit/pixel
	//		- 110: 18-bit/pixel
	//		- 111: 24-bit/pixel
	tft_write((uint32_t) 0x3A, true);
	tft_write((uint32_t) (pixel_format & 0x0070), false);
}

void tft_write_memory_continue( )
{
	//Transfer image information from the host processor interface to the SSD1963 from the last Write Memory Continue, 0x3C or Write Memory Start, 0x2C.
	tft_write((uint32_t) 0x3C, true);	
}

void tft_read_memory_continue( )
{
	//Read image data from the SSD1963 to host processor continuing after the last Read Memory Continue, 0x3E or Read Memory Start, 0x2E.
	tft_write((uint32_t) 0x3E, true);		
}

void tft_set_tear_scanline( uint32_t scanline )
{
	uint32_t scanline_high = (scanline & 0xFF00) >> 8;
	uint32_t scanline_low = (scanline & 0x00FF);
	
	//TE signal is sent from the display module to the host processor when the display device refresh reaches the provided scanline, N.
	
	tft_write((uint32_t) 0x44, true);
	tft_write((uint32_t) scanline_high, false);
	tft_write((uint32_t) scanline_low, false);
}

////-------------------------------- pg. 21

uint32_t tft_get_scanline( )
{
	// Get the current scan line, N.
	
	uint32_t data;
	tft_write((uint32_t) 0x45, true);
	tft_delay(6);
	data = (tft_read() & 0xFF) << 8;
	tft_delay(2);
	data |= (tft_read() & 0xFF);	

	// Return value - current scanline

	return data;
}

void tft_read_ddb( DDB_STRUCT_TYPE * ddb )
{
	//Read the DDB (Device Descriptor Block) information of SSD1963.
	tft_write((uint32_t) 0x45, true);
	tft_delay(6);
	ddb->supplier_id = (tft_read() & 0xFF) << 8;	//always 0x01
	tft_delay(2);
	ddb->supplier_id |= (tft_read() & 0xFF);		//always 0x57
	tft_delay(2);
	ddb->product_id = (tft_read() & 0xFF);		//always 0x61
	tft_delay(2);
	ddb->revision = (tft_read() & 0x7);			//always 0x01
	tft_delay(2);
	ddb->exit_code = (tft_read() & 0xFF);		//always 0xFF
}

void tft_set_lcd_mode( LCD_MODE_STRUCT_TYPE * lcd_mode_val )		// 9.35 Set LCD Mode
{
	uint32_t hor_panel_size_high = (lcd_mode_val->hor_panel_size & 0x0700) >> 8;
	uint32_t hor_panel_size_low = (lcd_mode_val->hor_panel_size & 0x00FF);
	uint32_t ver_panel_size_high = (lcd_mode_val->ver_panel_size & 0x0700) >> 8;
	uint32_t ver_panel_size_low = (lcd_mode_val->ver_panel_size & 0x00FF);

	// Set the LCD panel mode (RGB TFT or TTL) and pad strength
	
	// - HPS: Horizontal Panel Size (11 bits)
	// - VPS: Vertical Panel Size (11 bits)

	// -> REFER to solomon_systech_ssd1963 datasheet for additional information
	tft_write((uint32_t) 0xB0, true);
	tft_write((uint32_t) (lcd_mode_val->tft_settings & 0x3F), false);
	tft_write((uint32_t) (lcd_mode_val->mode_and_type & 0xE0) , false);
	tft_write((uint32_t) hor_panel_size_high, false);
	tft_write((uint32_t) hor_panel_size_low, false);
	tft_write((uint32_t) ver_panel_size_high, false);
	tft_write((uint32_t) ver_panel_size_low, false);
	tft_write((uint32_t) (lcd_mode_val->rgb_sequence & 0x3F) , false);	
}

 void tft_get_lcd_mode( LCD_MODE_STRUCT_TYPE * lcd_mode_val )
{
	//Get the current LCD panel mode and resolution
	
	tft_write((uint32_t) 0xB1, true);
	tft_delay(6);
	lcd_mode_val->tft_settings = (tft_read() & 0x3F);
	tft_delay(2);
	lcd_mode_val->mode_and_type = (tft_read() & 0xE0);
	tft_delay(2);	
	lcd_mode_val->hor_panel_size = (tft_read() & 0x07) << 8;
	tft_delay(2);
	lcd_mode_val->hor_panel_size |= (tft_read() & 0xFF);
	tft_delay(2);
	lcd_mode_val->ver_panel_size = (tft_read() & 0x07) << 8;
	tft_delay(2);
	lcd_mode_val->ver_panel_size |= (tft_read() & 0xFF);
	tft_delay(2);	
	
	lcd_mode_val->rgb_sequence = (tft_read() & 0x3F);
	
}



//void set_hori_period( void );
//uint32_t get_hori_period( void );
//void set_vert_period( void );
//uint32_t get_vert_period( void );
//void set_gpio_conf( void );
//uint32_t get_gpio_conf( void );
//void set_gpio_value( void );
//uint32_t get_gpio_status( void );
//void set_post_proc( void );
//uint32_t get_post_proc( void );
//void set_pwm_conf( void );
//uint32_t get_pwm_conf( void );
//void set_lcd_gen0( void );
//uint32_t get_lcd_gen0( void );
//void set_lcd_gen1( void );
//uint32_t get_lcd_gen1( void );
//void set_lcd_gen2( void );
//uint32_t get_lcd_gen2( void );
//void set_lcd_gen3( void );
//uint32_t get_lcd_gen3( void );
//void set_gpio0_rop( void );
//uint32_t get_gpio0_rop( void );
//void set_gpio1_rop( void );
//uint32_t get_gpio1_rop( void );
//void set_gpio2_rop( void );
//uint32_t get_gpio2_rop( void );
////-------------------------------- pg. 22
//void set_gpio3_rop( void );
//uint32_t get_gpio3_rop( void );
//void set_dbc_conf( void );
//uint32_t get_dbc_conf( void );
//void set_dbc_th( void );
//uint32_t get_dbc_th( void );
//void set_pll( void );
//void set_pll_mn( void );
//uint32_t get_pll_mn( void );
//uint32_t get_pll_status( void );
//void set_deep_sleep( void );
//void set_lshift_freq( void );
//uint32_t get_lshift_freq( void );

void tft_set_pixel_data_interface( uint32_t pixel_data_intfc )
{
	//Set the pixel data format to 8-bit / 9-bit / 12-bit / 16-bit / 16-bit(565) / 18-bit / 24-bit in the parallel host processor interface
	//Pixel Data Interface Format (POR = 101)
	//000 - 8-bit
	//001 - 12-bit
	//010 - 16-bit packed
	//011 - 16-bit (565 format)
	//100 - 18-bit
	//101 - 24-bit
	//110 - 9-bit
	//Others - Reserved
	
	tft_write((uint32_t) 0xF0, true);
	tft_write((uint32_t) (pixel_data_intfc & 0x03), false);
}

uint32_t tft_get_pixel_data_interface( void )
{
	// Get the current pixel data format settings in the parallel host processor interface.
	
	uint32_t data;
	tft_write((uint32_t) 0xF1, true);
	tft_delay(6);
	data = (tft_read() & 0x07);

	// Return value - Pixel Data Interface Format (POR = 101)
	//000 - 8-bit
	//001 - 12-bit
	//010 - 16-bit packed
	//011 - 16-bit (565 format)
	//100 - 18-bit
	//101 - 24-bit
	//110 - 9-bit
	//Others - Reserved

	return data;
}
