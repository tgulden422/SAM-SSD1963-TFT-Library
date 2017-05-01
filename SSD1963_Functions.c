/*
 * SSD1963_Functions.c
 *
 * Created: 4/29/2017 11:58:58 AM
 *  Author: Chris Dickinson
 */ 

#include "SSD1963_Functions.h"

#pragma GCC optimize 0

//--------------	FUNCTION DEFINITION		--------------------
//====================================================================

//== ADDRESS MODE==

uint8_t page_address_order = 0;	//A[7] : Page address order (POR = 0)
								//0 - Top to bottom, pages transferred from SP (Start Page) to EP (End Page).
								//1 - Bottom to top, pages transferred from EP (End Page) to SP (Start Page).
uint8_t col_address_order = 0;	//A[6] : Column address order (POR = 0)
								//0 - Left to right, columns transferred from SC (Start Column) to EC (End Column).
								//1 - Right to left, columns transferred from EC (End Column) to SC (Start Column).
uint8_t col_order = 1;			//A[5] : Page / Column order (POR = 0)
								//0 - Normal mode, 1 - Reverse mode
uint8_t row_address_order = 0;	//A[4] : Line address order (POR = 0)
								//0 - LCD refresh from top line to bottom line.
								//1 - LCD refresh from bottom line to top line.
uint8_t rgb_bgr_order = 0;		//A[3] : RGB / BGR order (POR = 0)
								//0 - RGB, 1 - BGR	
uint8_t display_data = 0;		//A[2] : Display data latch data (POR = 0)
								//0 - LCD refresh from left side to right side
								//1 - LCD refresh from right side to left side.						
uint8_t flip_horizontal = 0;	//A[1] : Flip Horizontal (POR = 0)
								//0 - Normal, 1 - Flipped
uint8_t flip_vertical = 0;		//A[0] : Flip Vertical (POR = 0)
								//0 - Normal, 1 - Flipped

//== PIXEL FORMAT==

uint8_t pixel_format = 0x3;		//000 - 8-bit
								//001 - 12-bit
								//010 - 16-bit packed
								//011 - 16-bit (565 format)
								//100 - 18-bit
								//101 - 24-bit
								//110 - 9-bit
								//Others - Reserved

LCD_MODE_STRUCT_TYPE lcd_mode = {0x00, 0x00, 799, 479, 0x00};

void tft_init_configuration()
{
	tft_exit_sleep_mode();	
	
	tft_set_address_mode( page_address_order, col_address_order, col_order, row_address_order, rgb_bgr_order, display_data, flip_horizontal, flip_vertical );
	
	tft_set_partial_area(0, 0);
	tft_set_pixel_data_interface( (uint32_t) pixel_format );
	tft_set_gamma_curve(0);
	tft_set_lcd_mode(&lcd_mode);

//	tft_get_lcd_mode(&lcd_mode);	

//	tft_set_tear_on(1);	
//	tft_set_tear_off();	

	tft_exit_idle_mode();
	tft_set_display_on();
	tft_enter_normal_mode();
}

void tft_write_pixel_val( uint8_t R,
						  uint8_t G,
						  uint8_t B,
						  uint8_t pixel_format_def)
{
	switch (pixel_format_def)
	{
		case 0x0:	//000 - 8-bit
			tft_write((uint32_t) R, false);		//R7 R6 R5 R4 R3 R2 R1 R0
			tft_delay(1);
			tft_write((uint32_t) G, false);		//G7 G6 G5 G4 G3 G2 G1 G0
			tft_delay(1);
			tft_write((uint32_t) B, false);		//B7 B6 B5 B4 B3 B2 B1 B0
			tft_delay(1);
		break;

		case 0x1:	//001 - 12-bit
			tft_write(((uint32_t) R) << 4 | (((uint32_t) G) >> 4), false);	//R7 R6 R5 R4 R3 R2 R1 R0 G7 G6 G5 G4
			tft_delay(1);
			tft_write(((uint32_t) (G & 0xF)) << 8 | B, false);				//G3 G2 G1 G0 B7 B6 B5 B4 B3 B2 B1 B0
			tft_delay(1);
		break;		
		
		case 0x2:	//010 - 16-bit packed
			tft_write(((uint32_t) R) << 8 | (uint32_t) G, false);		//R7 R6 R5 R4 R3 R2 R1 R0 G7 G6 G5 G4 G3 G2 G1 G0
			tft_delay(1);
			tft_write(((uint32_t) B) << 8 | (uint32_t) R, false);		//B7 B6 B5 B4 B3 B2 B1 B0 R7 R6 R5 R4 R3 R2 R1 R0 
			tft_delay(1);
			tft_write(((uint32_t) G) << 8 | (uint32_t) B, false);		//G7 G6 G5 G4 G3 G2 G1 G0 B7 B6 B5 B4 B3 B2 B1 B0	
			tft_delay(1);		
		break;
		
		case 0x3:	//011 - 16-bit (565 format)
			tft_write( (((uint32_t) (R & 0xF8)) << 8) | (((uint32_t) (G & 0xFC)) << 3) | (((uint32_t) B) >> 3), false);		//R7 R6 R5 R4 R3 G7 G6 G5 G4 G3 G2 B7 B6 B5 B4 B3		
			tft_delay(1);
		break;		
			
		case 0x4:	//100 - 18-bit
			tft_write( (((uint32_t) (R & 0xFC)) << 10) | (((uint32_t) (G & 0xFC)) << 4) | (((uint32_t) B) >> 2), false);		//R7 R6 R5 R4 R3 R2 G7 G6 G5 G4 G3 G2 B7 B6 B5 B4 B3 B2	
			tft_delay(1);
		break;
		
		case 0x5:	//101 - 24-bit
			tft_write( (((uint32_t) R) << 16) | (((uint32_t) G) << 8) | ((uint32_t) B), false);		//R7 R6 R5 R4 R3 R2 R1 R0 G7 G6 G5 G4 G3 G2 G1 G0 B7 B6 B5 B4 B3 B2 B1 B0
			tft_delay(1);
		break;
		
		case 0x6:	//110 - 9-bit
			tft_write(((uint32_t) (R & 0xFC)) << 1 | (((uint32_t) G) >> 5), false);	//R7 R6 R5 R4 R3 R2 G7 G6 G5
			tft_delay(1);
			tft_write(((uint32_t) (G & 0x1C)) << 4 | (((uint32_t) B) >> 2), false);	//G4 G3 G2 B7 B6 B5 B4 B3 B2
			tft_delay(1);
		break;
	
		default:	//Others - Reserved
		break;
	}
}

void tft_write_screen()
{
	uint32_t i;
	uint32_t j;	
	uint32_t start_x = 0;
	uint32_t stop_x = lcd_mode.hor_panel_size;	
	uint32_t start_y = 0;
	uint32_t stop_y = lcd_mode.ver_panel_size;	
	
	tft_set_page_address(start_y, stop_y);		
	tft_set_column_address(start_x, stop_x);
	tft_write_memory_start();
	
	for (j = start_y; j <= stop_y; j++ )
	{
		for (i = start_x; i <= stop_x; i++ )
		{
			switch((i / 100))
			{
				case 0:
					tft_write_pixel_val( 0xC0, 0xC0, 0xC0, pixel_format);				
				break;
				case 1:
					tft_write_pixel_val( 0xFF, 0xFF, 0x00, pixel_format);					
				break;
				case 2:
					tft_write_pixel_val( 0x00, 0xFF, 0xFF, pixel_format);				
				break;
				case 3:
					tft_write_pixel_val( 0x00, 0xFF, 0x00, pixel_format);					
				break;
				case 4:
					tft_write_pixel_val( 0xFF, 0x00, 0xFF, pixel_format);				
				break;
				case 5:
					tft_write_pixel_val( 0xFF, 0x00, 0x00, pixel_format);					
				break;
				case 6:
					tft_write_pixel_val( 0x00, 0x00, 0xFF, pixel_format);				
				break;
				case 7:
					tft_write_pixel_val( 0x00, 0x00, 0x00, pixel_format);					
				break;				
			}
			tft_write_memory_continue();			
		}
	}
	tft_nop();
	//tft_set_scroll_area(5, 5, 5);
	//tft_set_scroll_start(6);
	
	
}




