/**************************************************************
	This is a driver for New Haven Display controller
	boards featuring an SSD1963 controller chip.
	
	This code resource runs on ASF for Atmel SAM MCU's.
	It depends on IOPORT service, which must be included
	in ASF wizard in Atmel Studio.
**************************************************************/

#ifndef	SAM_1963_DEFINED
	#define SAM_1963_DEFINED
	
	//--------------	DEPENDENCIES 	---------------------------
	#include <ioport.h>

	//--------------	CODE CONFIG		---------------------------
	#define	CLOCK_DELAY		(	0x000000FF	)	//Provides crude delay interval for set/reset operation to be registered by controller if host MCU clock much faster
	#define	NS_DELAY		(	0x00000001	)	//loop delay for 1 ns
	#define RST_TFT_ON_INIT						//Comment out to disable controller reset on init

	//--------------	IOPORT PIN SELECT		-------------------
	#define	TFT_LOW_PORT	IOPORT_PIOA			//ASF ports
	#define	TFT_HIGH_PORT	IOPORT_PIOD
	#define TFT_LOW_MASK	(	0x1FE00000	) 	//Masks for offset port pin block
	#define TFT_HIGH_MASK	(	0x007F8000	) 
	#define TFT_LOW_OFFSET	(	21	)			//Pin offset from 0 for port block
	#define	TFT_HIGH_OFFSET	(	15	)

	#define BL_EN			PIO_PC12_IDX	//OK	//Backlight enable
	#define DC_SEL			PIO_PA29_IDX	//OK	//Data/command select, Low = Command, High = Data
	#define	WR_8080			PIO_PA10_IDX	//OK	//Write enable for 8080 mode, active low (Read/Write select in 6800 mode, Low = Write, High = Read)
	#define	RD_8080			PIO_PA12_IDX	//OK	//Read enable for 8080 mode, active low (Enable bit in 6800 mode)
	#define	CS_BIT			PIO_PC14_IDX	//OK	//Channel select, active low
	#define	TFT_ON			PIO_PA18_IDX	//OK	//Display power select, ( DISP )
	#define	TFT_RST			PIO_PA31_IDX	//OK	//Controller reset, active low

	// Data Port Low Byte [PA:12~28]
	#define DATA_0			PIO_PA21_IDX	//OK
	#define	DATA_1			PIO_PA22_IDX	//OK
	#define	DATA_2			PIO_PA23_IDX	//OK
	#define	DATA_3			PIO_PA24_IDX	//OK
	#define DATA_4			PIO_PA25_IDX	//OK
	#define	DATA_5			PIO_PA26_IDX	//OK
	#define	DATA_6			PIO_PA27_IDX	//OK
	#define	DATA_7			PIO_PA28_IDX	//OK

	// Data Port High Byte [PD:15~22]
	#define DATA_8			PIO_PD15_IDX	//OK
	#define	DATA_9			PIO_PD16_IDX	//OK
	#define	DATA_10			PIO_PD17_IDX	//OK
	#define	DATA_11			PIO_PD18_IDX	//OK
	#define DATA_12			PIO_PD19_IDX	//OK
	#define	DATA_13			PIO_PD20_IDX	//OK
	#define	DATA_14			PIO_PD21_IDX	//OK
	#define	DATA_15			PIO_PD22_IDX	//OK

	//--------------	FUNCTION PROTOTYPES		--------------------
	void tft_delay( uint32_t );
	void tft_write( uint32_t , bool );
	uint32_t tft_read( void );
	
	void tft_init( void );
	void data_port_init( void );
	void ctl_bits_init( void );
	void write_data_port( uint32_t );
	uint32_t read_data_port( void );

	// Pin direction setting
	void set_write_enable( void );
	void set_read_enable( void );
	
//	void set_mode_cmd( void );
	#define set_mode_cmd() ioport_set_pin_level(DC_SEL, 0) //Sets command mode
	//OK
	
//	void set_mode_data( void );
	#define set_mode_data() ioport_set_pin_level(DC_SEL, 1) //Sets data mode
	//OK

//	void tft_select( bool cs_state );
	#define tft_select(cs_state) ioport_set_pin_level(CS_BIT, cs_state) //Selects or deselects the display based on cs_state (8080 Mode Timing Diagram)
	
//	void bl_enable(	bool bl_state );
	#define bl_enable(bl_state)	ioport_set_pin_level(BL_EN, bl_state)	//Enables or disables backlight based on bl_state
	//OK

//	void tft_enable( bool en_state );
	#define tft_enable(en_state) ioport_set_pin_level(TFT_ON, en_state) //Turns TFT on or off based on en_state
	
	void rst_tft( void );

#endif
