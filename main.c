/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "SAM_1963.h"
#include "SSD1963_CommandLib.h"
#include "SSD1963_Functions.h"

#pragma GCC optimize 0

int main (void)
{
	volatile uint32_t data1 = 0;
	volatile uint32_t data2 = 0;	
	volatile uint32_t data3 = 0;
	volatile uint32_t data4 = 0;		

	/* Insert system clock initialization code here (sysclk_init()). */
//	sysclk_init();
	
	board_init();
	ioport_init(); //ASF ioport initialization

	tft_init(); //SSD_1963 library port initialization (do ASF ioport_init first!)
	
	tft_init_configuration();

	tft_write_screen();
	
	//tft_enter_invert_mode();	
	
	/* Insert application code here, after the board has been initialized. */
	
	while (1)
	{
		tft_enter_invert_mode();
		tft_delay(10);
		tft_exit_invert_mode();
		tft_delay(10);		
	}
	
	return data1 + data2 + data3 + data4 ;
}
