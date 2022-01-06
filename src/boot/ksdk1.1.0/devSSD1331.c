#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 11),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}



int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	warpEnableSPIpins();
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);


	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 11u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color, Horizantal address increment, RAM column 0 to 95 maps to Pin Seg (SA, SB, SC) 95 to 0
							// 65K colour format, Enable COM Split Odd Even, Scan from COM[N-1] to COM 0
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);				// Set display start line register to 0
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);				// Vertical offset by COM set to 0
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);				//Select  external Vcc supply
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);				//Disable power save mode
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0xF1);				// Phase 1 set to 1 DCLK, Phase 2 set to 3 DCLK
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);				// Pre-charge level A
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x64);				// Pre-charge level B
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);				// Pre-charge level C
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3E);				//Precharge voltage level
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);				// COM deselect voltage set to 0.88VCC
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x0F);				// 16/16 attenuation for master current
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0xFF);				// Contrast for colour a set to 145
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0xFF);				// Contrast for colour B set to 80
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0xFF);				// Contrast for colour C set to 125
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
	
	//writeCommand(0xB8); //Set Grayscale levels
	//writeCommand(0x01);
	//writeCommand(0x
	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL); //0x26
	writeCommand(0x01); // Enable fill for draw rectangle

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);  //0x25
	writeCommand(0x00);			//Column address of start
	writeCommand(0x00);			//Row address of start
	writeCommand(0x5F);			//Column end of start 95d
	writeCommand(0x3F);			//Row adress of end	63d



	/*
	 *	Any post-initialization drawing commands go here.
	 */
	//...

        writeCommand(kSSD1331CommandDRAWRECT);  //0x22 Draw rectanngle
        writeCommand(0x00); // Start C
        writeCommand(0x00); // Start R
        writeCommand(0x5F); // End C
        writeCommand(0x3F); // End R
        writeCommand(0x3F); // Color C of line (max 0x1F 31), 8, F, 1F, 2F, 3F
        writeCommand(0x3F); // Color B of line (max 0X3F 63)
        writeCommand(0x3F); // Color A of line
        writeCommand(0x3F); // Color C of fill
        writeCommand(0x3F); // Color B of fill
        writeCommand(0x3F); // Color A of fill

	return 0;
}
