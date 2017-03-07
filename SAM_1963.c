/**************************************************************
	This is a driver for New Haven Display controller
	boards featuring an SSD1963 controller chip.
	
	This code resource runs on ASF for Atmel SAM MCU's.
	It depends on IOPORT service, which must be included
	in ASF wizard in Atmel Studio.
**************************************************************/

//--------------	CODE CONFIG		---------------------------
#define	CLOCK_DELAY		(	0x00FFFFFF	)	//Provides crude delay interval for set/reset operation to be registered by controller if host MCU clock much faster
#define RST_TFT_ON_INIT						//Comment out to disable controller reset on init

//--------------	IOPORT PIN SELECT		-------------------
#define	TFT_LOW_PORT	IOPORT_PIOA			//ASF ports
#define	TFT_HIGH_PORT	IOPORT_PIOD
#define TFT_LOW_MASK	(	0x000000FF	) 	//Masks for offset port pin block
#define TFT_HIGH_MASK	(	0x007F8000	) 
#define TFT_LOW_OFFSET	(	0	)			//Pin offset from 0 for port block
#define	TFT_HIGH_OFFSET	(	15	)

#define BL_EN			PIO_PB0_IDX			//Backlight enable
#define DC_SEL			PIO_PB1_IDX			//Data/command select, Low = Command, High = Data
#define	WR_8080			PIO_PB2_IDX			//Write enable for 8080 mode, active low (Read/Write select in 6800 mode, Low = Write, High = Read)
#define	RD_8080			PIO_PB3_IDX			//Read enable for 8080 mode, active low (Enable bit in 6800 mode)
#define	CS_BIT			PIO_PB4_IDX			//Channel select, active low

#define	TFT_ON			PIO_PB5_IDX			//Display power select
#define	TFT_RST			PIO_PB6_IDX			//Controller reset, active low

//--------------	FUNCTION PROTOTYPES		--------------------


//--------------	FUNCTION DEFINITION		--------------------
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
	ioport_set_pin_level(DC_SEL, 0);
	
	//Disable channel select
	ioport_set_pin_level(CS_BIT, 1);
	
	//Reset WR# and RD# high
	ioport_set_pin_level(WR_8080, 1);
	ioport_set_pin_level(RD_8080, 1);
	
#ifdef	RST_TFT_ON_INIT
	//Reset TFT controller
	rst_tft();
#endif
	
	//Turn on TFT
	ioport_set_pin_level(TFT_ON, 1);
}

void write_data_port( uint32_t data_out )
{
	//Writes word to data pins
	uint32_t buffer_low;
	uint32_t buffer_high;
	
	buffer_low = TFT_LOW_MASK & data_out;
	buffer_high = TFT_HIGH_MASK & (data_out << TFT_HIGH_OFFSET);
	
	ioport_set_port_level(TFT_LOW_PORT, TFT_LOW_MASK, buffer_low);
	ioport_set_port_level(TFT_HIGH_PORT, TFT_HIGH_MASK, buffer_high);
}

void set_mode_cmd( void )
{
	//Sets command mode
	ioport_set_pin_level(DC_SEL, 0);
}

void set_mode_data( void )
{
	//Sets data mode
	ioport_set_pin_level(DC_SEL, 1);
}

void bl_enable(	bool bl_state )
{
	//Enables or disable backlight based on bl_state
	ioport_set_pin_level(BL_EN, bl_state);
}

void tft_enable( bool en_state )
{
	//Turns TFT on or off based on en_state
	ioport_set_pin_level(TFT_ON, en_state);
}

void rst_tft( void )
{
	//Reset TFT controller
	ioport_set_pin_level(TFT_RST, 1);
	
	//Crude delay to ensure reference clock can catch up with host MCU clock
	volatile int count;
	for(count = 0; count < CLOCK_DELAY; count++);
	
	ioport_set_pin_level(TFT_RST, 0);
}