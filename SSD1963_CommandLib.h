/*
 * SSD1963_CommandLib.h
 *
 * Created: 4/29/2017 11:58:58 AM
 *  Author: Chris Dickinson
 */ 

#ifndef	SSD1963_CMDLIB_DEFINED
	#define SSD1963_CMDLIB_DEFINED
	
	//--------------	DEPENDENCIES 	---------------------------
	#include "SAM_1963.h"
	#include "SSD1963_Data_Transaction.h"

	//--------------	CODE CONFIG		---------------------------


	typedef struct  
	{
		uint32_t supplier_id;
		uint32_t product_id;
		uint32_t revision;
		uint32_t exit_code;		
	} DDB_STRUCT_TYPE;
	
	typedef struct
	{
		uint32_t tft_settings;
		uint32_t mode_and_type;
		uint32_t hor_panel_size;
		uint32_t ver_panel_size;
		uint32_t rgb_sequence;
	} LCD_MODE_STRUCT_TYPE;	

//	#define BL_EN			//PIO_PB0_IDX			//Backlight enable
//	#define DC_SEL			//PIO_PB1_IDX			//Data/command select, Low = Command, High = Data
//	#define	WR_8080			//PIO_PB2_IDX			//Write enable for 8080 mode, active low (Read/Write select in 6800 mode, Low = Write, High = Read)
//	#define	RD_8080			//PIO_PB3_IDX			//Read enable for 8080 mode, active low (Enable bit in 6800 mode)
//	#define	CS_BIT			//PIO_PB4_IDX			//Channel select, active low

//	#define	TFT_ON			//PIO_PB5_IDX			//Display power select
//	#define	TFT_RST			//PIO_PB6_IDX			//Controller reset, active low

	//--------------	FUNCTION PROTOTYPES		--------------------
	//-------------------------------- pg. 20
	void tft_nop( void ); // <-------
	void tft_soft_reset( void ); // <-------
	uint32_t tft_get_power_mode( void ); // <-------
	uint32_t tft_get_address_mode( void ); // <-------
	uint32_t tft_get_pixel_format( void ); // <-------
	uint32_t tft_get_display_mode( void ); // <-------
	uint32_t tft_get_signal_mode( void ); // <-------
	void tft_enter_sleep_mode( void ); // <-------
	void tft_exit_sleep_mode( void ); // <-------
	void tft_enter_partial_mode( void ); // <-------
	void tft_enter_normal_mode( void ); // <-------
	void tft_exit_invert_mode( void ); // <-------
	void tft_enter_invert_mode( void ); // <-------
	void tft_set_gamma_curve( uint32_t gamma_curve ); // <-------
	void tft_set_display_off( void ); // <-------
	void tft_set_display_on( void ); // <-------
	void tft_set_column_address( uint32_t, uint32_t ); // <-------
	void tft_set_page_address( uint32_t, uint32_t ); // <-------
	void tft_write_memory_start( void ); // <-------
	void tft_read_memory_start( void ); // <-------
	void tft_set_partial_area( uint32_t, uint32_t ); // <-------
	void tft_set_scroll_area( uint32_t, uint32_t, uint32_t ); // <-------
	void tft_set_tear_off( void ); // <-------
	void tft_set_tear_on( uint32_t ); // <-------
	void tft_set_address_mode( uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t  ); // <-------
	void tft_set_scroll_start( uint32_t ); // <-------
	void tft_exit_idle_mode( void ); // <-------
	void tft_enter_idle_mode( void ); // <-------
	void tft_set_pixel_format( uint32_t ); // <-------
	void tft_write_memory_continue( void ); // <-------
	void tft_read_memory_continue( void ); // <-------
	void tft_set_tear_scanline( uint32_t ); // <-------
	//-------------------------------- pg. 21
	uint32_t tft_get_scanline( void ); // <-------
	void tft_read_ddb( DDB_STRUCT_TYPE *); // <-------
	void tft_set_lcd_mode( LCD_MODE_STRUCT_TYPE * ); // <-------
	void tft_get_lcd_mode( LCD_MODE_STRUCT_TYPE * ); // <-------
	
	void tft_set_hori_period( void );
	uint32_t tft_get_hori_period( void );
	void tft_set_vert_period( void );
	uint32_t tft_get_vert_period( void );
	void tft_set_gpio_conf( void );
	uint32_t tft_get_gpio_conf( void );
	void tft_set_gpio_value( void );
	uint32_t tft_get_gpio_status( void );
	void tft_set_post_proc( void );
	uint32_t tft_get_post_proc( void );
	void tft_set_pwm_conf( void );
	uint32_t tft_get_pwm_conf( void );
	void tft_set_lcd_gen0( void );
	uint32_t tft_get_lcd_gen0( void );
	void tft_set_lcd_gen1( void );
	uint32_t tft_get_lcd_gen1( void );
	void tft_set_lcd_gen2( void );
	uint32_t tft_get_lcd_gen2( void );
	void tft_set_lcd_gen3( void );
	uint32_t tft_get_lcd_gen3( void );
	void tft_set_gpio0_rop( void );
	uint32_t tft_get_gpio0_rop( void );
	void tft_set_gpio1_rop( void );
	uint32_t tft_get_gpio1_rop( void );
	void tft_set_gpio2_rop( void );
	uint32_t tft_get_gpio2_rop( void );
	//-------------------------------- pg. 22
	void tft_set_gpio3_rop( void );
	uint32_t tft_get_gpio3_rop( void );
	void tft_set_dbc_conf( void );
	uint32_t tft_get_dbc_conf( void );
	void tft_set_dbc_th( void );
	uint32_t tft_get_dbc_th( void );
	void tft_set_pll( void );
	void tft_set_pll_mn( void );
	uint32_t tft_get_pll_mn( void );
	uint32_t tft_get_pll_status( void );
	void tft_set_deep_sleep( void );
	void tft_set_lshift_freq( void );
	uint32_t tft_get_lshift_freq( void );
	
	void tft_set_pixel_data_interface( uint32_t );
	uint32_t tft_get_pixel_data_interface( void );


#endif
