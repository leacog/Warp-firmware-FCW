/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: See git blame.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
#include "glaux.h"
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "fsl_adc16_driver.h"
#include "ADC.h"
#include "FFT.h"
#include "PWM.h"
#include "complex.h"


volatile bool						gWarpBooted				= false;
volatile WarpModeMask					gWarpMode				= kWarpModeDisableAdcOnSleep;
volatile uint32_t					gWarpUartTimeoutMilliseconds		= kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t					gWarpMenuPrintDelayMilliseconds		= kWarpDefaultMenuPrintDelayMilliseconds;
char							gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];

static void						lowPowerPinStates(void);
static void						dumpProcessorState(void);
int freq = 0;

uint16_t rollingAverage(uint16_t newVal, uint16_t * rollArray, uint8_t * idx);
void SetTrebbleRGB(uint16_t * RGBvals);
void SetBaseRGB(uint16_t * RGBvals);
void printFFT(int complex * x, int n);
#define rollNumber 4

/*
 *	Derived from KSDK power_manager_demo.c BEGIN>>>
 */
clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}

/*
 *	LLW_IRQHandler override. Since FRDM_KL03Z48M is not defined,
 *	according to power_manager_demo.c, what we need is LLW_IRQHandler.
 *	However, elsewhere in the power_manager_demo.c, the code assumes
 *	FRDM_KL03Z48M _is_ defined (e.g., we need to use LLWU_IRQn, not
 *	LLW_IRQn). Looking through the code base, we see in
 *
 *		ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
 *
 *	that the startup initialization assembly requires LLWU_IRQHandler,
 *	not LLW_IRQHandler. See power_manager_demo.c, circa line 216, if
 *	you want to find out more about this dicsussion.
 */
void
LLWU_IRQHandler(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

volatile bool adcRdyFlag = 0;
volatile uint16_t adcRawValue = 0;
void ADC0_IRQHandler(void)
{
	adcRdyFlag = true;
	adcRawValue = ADC16_DRV_GetConvValueRAW(0, 0);
}

/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
void
BOARD_SW_LLWU_IRQ_HANDLER(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t
callback0(power_manager_notify_struct_t *  notify, power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}


void
dumpProcessorState(void)
{
	uint32_t	cpuClockFrequency;

	CLOCK_SYS_GetFreq(kCoreClock, &cpuClockFrequency);
	warpPrint("\r\n\n\tCPU @ %u KHz\n", (cpuClockFrequency / 1000));
	warpPrint("\r\tCPU power mode: %u\n", POWER_SYS_GetCurrentMode());
	warpPrint("\r\tCPU clock manager configuration: %u\n", CLOCK_SYS_GetCurrentConfiguration());
	warpPrint("\r\tRTC clock: %d\n", CLOCK_SYS_GetRtcGateCmd(0));
	warpPrint("\r\tSPI clock: %d\n", CLOCK_SYS_GetSpiGateCmd(0));
	warpPrint("\r\tI2C clock: %d\n", CLOCK_SYS_GetI2cGateCmd(0));
	warpPrint("\r\tLPUART clock: %d\n", CLOCK_SYS_GetLpuartGateCmd(0));
	warpPrint("\r\tPORT A clock: %d\n", CLOCK_SYS_GetPortGateCmd(0));
	warpPrint("\r\tPORT B clock: %d\n", CLOCK_SYS_GetPortGateCmd(1));
	warpPrint("\r\tFTF clock: %d\n", CLOCK_SYS_GetFtfGateCmd(0));
	warpPrint("\r\tADC clock: %d\n", CLOCK_SYS_GetAdcGateCmd(0));
	warpPrint("\r\tCMP clock: %d\n", CLOCK_SYS_GetCmpGateCmd(0));
	warpPrint("\r\tVREF clock: %d\n", CLOCK_SYS_GetVrefGateCmd(0));
	warpPrint("\r\tTPM clock: %d\n", CLOCK_SYS_GetTpmGateCmd(0));
}

void
warpPrint(const char *fmt, ...)
{
	int	fmtlen;
	va_list	arg;

	/*
	 *	We use an ifdef rather than a C if to allow us to compile-out
	 *	all references to SEGGER_RTT_*printf if we don't want them.
	 *
	 *	NOTE: SEGGER_RTT_vprintf takes a va_list* rather than a va_list
	 *	like usual vprintf. We modify the SEGGER_RTT_vprintf so that it
	 *	also takes our print buffer which we will eventually send over
	 *	BLE. Using SEGGER_RTT_vprintf() versus the libc vsnprintf saves
	 *	2kB flash and removes the use of malloc so we can keep heap
	 *	allocation to zero.
	 */
	#if (WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF)
		/*
		 *	We can't use SEGGER_RTT_vprintf to format into a buffer
		 *	since SEGGER_RTT_vprintf formats directly into the special
		 *	RTT memory region to be picked up by the RTT / SWD mechanism...
		 */
		va_start(arg, fmt);
		fmtlen = SEGGER_RTT_vprintf(0, fmt, &arg, gWarpPrintBuffer, kWarpDefaultPrintBufferSizeBytes);
		va_end(arg);

		if (fmtlen < 0)
		{
			SEGGER_RTT_WriteString(0, gWarpEfmt);

			#if (WARP_BUILD_ENABLE_DEVBGX)
				if (gWarpBooted)
				{
					WarpStatus	status;

					enableLPUARTpins();
					initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
					status = sendBytesToUART((uint8_t *)gWarpEfmt, strlen(gWarpEfmt)+1);
					if (status != kWarpStatusOK)
					{
						SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
					}
					disableLPUARTpins();

					/*
					 *	We don't want to deInit() the BGX since that would drop
					 *	any remote terminal connected to it.
					 */
					//deinitBGX();
				}
			#endif

			return;
		}

		/*
		 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
		 */
		#if (WARP_BUILD_ENABLE_DEVBGX)
			if (gWarpBooted)
			{
				WarpStatus	status;

				enableLPUARTpins();
				initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);

				status = sendBytesToUART((uint8_t *)gWarpPrintBuffer, max(fmtlen, kWarpDefaultPrintBufferSizeBytes));
				if (status != kWarpStatusOK)
				{
					SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
				}
				disableLPUARTpins();

				/*
				 *	We don't want to deInit() the BGX since that would drop
				 *	any remote terminal connected to it.
				 */
				//deinitBGX();
			}
		#endif
	#else
		/*
		 *	If we are not compiling in the SEGGER_RTT_printf,
		 *	we just send the format string of warpPrint()
		 */
		SEGGER_RTT_WriteString(0, fmt);

		/*
		 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
		 */
		#if (WARP_BUILD_ENABLE_DEVBGX)
			if (gWarpBooted)
			{
				WarpStatus	status;

				enableLPUARTpins();
				initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
				status = sendBytesToUART(fmt, strlen(fmt));
				if (status != kWarpStatusOK)
				{
					SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
				}
				disableLPUARTpins();

				/*
				 *	We don't want to deInit() the BGX since that would drop
				 *	any remote terminal connected to it.
				 */
				//deinitBGX();
			}
		#endif
	#endif

	return;
}
void
	lowPowerPinStates(void)
	{
		/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state. We choose
		 *	to set them all to '0' since it happens that the devices we want to keep
		 *	deactivated (SI4705) also need '0'.
		 */

		/*
		 *			PORT A
		 */
		/*
		 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

		/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
		 *
		 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

		/*
		 *	Disable PTA5
		 *
		 *	NOTE: Enabling this significantly increases current draw
		 *	(from ~180uA to ~4mA) and we don't need the RTC on revC.
		 *
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

		/*
		 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
		 *
		 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
		 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

		/*
		 *	NOTE: The KL03 has no PTA10 or PTA11
		 */
		//PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);


		/*
		 *			PORT B
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);
	}
int
main(void)
{
	WarpStatus				status;
	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	/*
	 *	We use this as a template later below and change the .mode fields for the different other modes.
	 */
	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: POWER_SYS_SetMode() depends on this order
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure		powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};

	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);

	/*
	 *	Set board crystal value (Warp revB and earlier).
	 */
	g_xtal0ClkFreq = 32768U;

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	/*
	 *	When booting to CSV stream, we wait to be up and running as soon as possible after
	 *	a reset (e.g., a reset due to waking from VLLS0)
	 */
	if (!WARP_BUILD_BOOT_TO_CSVSTREAM)
	{
		warpPrint("\n\n\n\rBooting Warp");
	}

	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM, /* The default value of this is defined in fsl_clock_MKL03Z4.h as 2 */
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));

	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);

	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	if (WARP_BUILD_BOOT_TO_VLPR)
	{
		warpPrint("About to switch CPU to VLPR mode... ");
		status = warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
		if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
		{
			warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR() failed...\n");
		}
		warpPrint("done.\n\r");
	}

	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	warpPrint("About to GPIO_DRV_Init()... ");
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	warpPrint("done.\n");

	/*
	 *	Make sure the SWD pins, PTA0/1/2 SWD pins in their ALT3 state (i.e., as SWD).
	 *
	 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);
	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	warpPrint("About to lowPowerPinStates()... ");
	lowPowerPinStates();
//	PORT_HAL_SetMuxMode(PORTB_BASE, 10u,kPortMuxAlt2); // Set PTB10 up for TPM0_CH1
//	PORT_HAL_SetMuxMode(PORTB_BASE, 11u,kPortMuxAlt2); // Set PTB7 up for TPM1_CH0
//	PORT_HAL_SetMuxMode(PORTB_BASE, 13u,kPortMuxAlt2); // Set PTB13 up for TPM1_CH1
	PORT_HAL_SetMuxMode(PORTA_BASE, 5u,kPortMuxAlt2); // Set PTA5 up for TPM0_CH1
	PORT_HAL_SetMuxMode(PORTA_BASE, 6u,kPortMuxAlt2); // Set PTA6 up for TPM1_CH0
	PORT_HAL_SetMuxMode(PORTB_BASE, 6u,kPortMuxAlt2); // Set PTB6 up for TPM1_CH1
	warpPrint("done.\n");

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		blinkLED(kGlauxPinLED);
		blinkLED(kGlauxPinLED);
		blinkLED(kGlauxPinLED);

		USED(disableTPS62740);
		USED(enableTPS62740);
		USED(setTPS62740CommonControlLines);
	#endif

	/*
	 *	At this point, we consider the system "booted" and, e.g., warpPrint()s
	 *	will also be sent to the BLE if that is compiled in.
	 */
	gWarpBooted = true;
	warpPrint("Boot done...\n");
	OSA_TimeDelay(1000);
	warpPrint("Starting program! \n");
	
	// ----- Set Power Mode ---
	
		
	warpSetLowPowerMode(kWarpPowerModeRUN, 0);
	if (status != kWarpStatusOK)
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeRUN, 0)() failed...\n");
	}
	

	dumpProcessorState();

	// ----- Init PWM -------
	
	TPM_init(0);
	TPM_init(1);
	PWM_init(pwm_R);
	PWM_init(pwm_B);
	PWM_init(pwm_G);
	PWM_SetDuty(pwm_R, 1024);
	PWM_SetDuty(pwm_B, 2024);
	PWM_SetDuty(pwm_G, 3024);
		

	// ----- Init ADC -------
	uint32_t instance = 0;
	uint32_t chnGroup = 0;
	uint8_t  chn      = 2u; //Sets ADC channel up to. PTA9 = 2, PB12 = 0, PT0 = 15
	//PORT_HAL_SetMuxMode(PORTA_BASE, 12u,kPortMuxAsGpio); // Set PTA12 for ADC
	ADC16_init_continuous(instance, chnGroup, chn);
	warpPrint("\n Set up ADC");
	uint32_t ADC_time = OSA_TimeGetMsec();
	int loopCount = 0;
	while(OSA_TimeGetMsec() - ADC_time < 1000){
		if(adcRdyFlag){
			loopCount++;
			adcRdyFlag = false;
		}
	}
	freq = loopCount;
	warpPrint("\nADC frequency: %u\n", (uint32_t)(freq));

	// ----- Init FFT -----
	uint8_t nPoint = 32;
	uint8_t sampleIdx = 0;
	uint8_t slowSampleIdx = 0;
	uint8_t slowSampleDivider = 4;
	int sampleBuffer[nPoint];
       	int slowSampleBuffer[nPoint];	
	int complex fftBuffer[nPoint];
	
	while (1)
	{
		if(adcRdyFlag){
			int newSample = ADC16_DRV_ConvRAWData(adcRawValue, false, kAdcResolutionBitOfSingleEndAs12);
			sampleBuffer[sampleIdx] = newSample - 2048;
			if( (sampleIdx % slowSampleDivider) == 0){
				slowSampleBuffer[slowSampleIdx] = newSample - 2048;
				slowSampleIdx++;
			}
			adcRdyFlag = false;
			sampleIdx++;
			/*
			if (sampleIdx == nPoint){
				bufferFilled = true;
				sampleIdx = 0;
			}
			*/
		}
		if(sampleIdx >= nPoint){
			uint32_t startTime, stopTime;
			startTime =OSA_TimeGetMsec();
			applyWindow32(&sampleBuffer[0]);
			for(int i = 0; i<nPoint; i++){
				fftBuffer[i] = sampleBuffer[i] + 0*I;
				//warpPrint("\n%u",sampleBuffer[i]);
				//bufferFilled = false;
			}
			FFT(&fftBuffer[0], nPoint);
			//printFFT(&fftBuffer[0], nPoint);
			uint16_t RGBvalues [3];
			octaves(&fftBuffer[0], &RGBvalues[0]);
			SetTrebbleRGB(&RGBvalues[0]);     //Turn on new values for leds
			sampleIdx = 0;
			stopTime = OSA_TimeGetMsec();
			
			//warpPrint("\nFFT-TIME: %u", (uint32_t)((stopTime-startTime)));
			
		} 
		
		if (slowSampleIdx >= nPoint) {  //Should time this function to see how long it takes. 
			//applyWindow32(&slowSampleBuffer[0]);
			for(int i = 0; i<nPoint; i++){
				fftBuffer[i] = slowSampleBuffer[i] + 0*I;
				//warpPrint("\n%u",sampleBuffer[i]);
				//bufferFilled = false;
			}
			FFT(&fftBuffer[0], nPoint);
			//printFFT(&fftBuffer[0], nPoint);
			//for (int i=0; i<nPoint; i++){
			//	warpPrint("\n%d", slowSampleBuffer[i]);
			//}
			uint16_t RGBvalues [3];
			octaves(&fftBuffer[0], &RGBvalues[0]);
			SetBaseRGB(&RGBvalues[0]);     //Turn on new values for leds
			slowSampleIdx = 0;
		}
		/*
		warpPrint("\nFFTTIME: %u", (uint32_t)((stopTime-startTime)));

		for(int ij = 0; ij < n; ij++){
			warpPrint("\nresult: RE[%d] - IM[%d]", (int)creal(xarray[ij]), (int)cimag(xarray[ij]));
		}
		*/
	}	
	return 0;
}


uint8_t Ridx = 0;//, Gidx = 0, Bidx =0;

//#define PRINT_RGB 1


void SetTrebbleRGB(uint16_t * RGBvals){
	/*
	static uint16_t Rroll[rollNumber+1]; 
	static uint16_t Groll[rollNumber+1]; 
	static uint16_t Broll[rollNumber+1];
	
	PWM_SetDuty(pwm_R, 4096 - (rollingAverage(RGBvals[0], &Rroll[0], &Ridx)));
	PWM_SetDuty(pwm_G, 4096 - (rollingAverage(RGBvals[1], &Groll[0], &Gidx)));
	PWM_SetDuty(pwm_B, 4096 - (rollingAverage(RGBvals[2], &Broll[0], &Bidx)));
	#ifdef PRINT_RGB
	for(int i = 0; i<3; i++){
		warpPrint("\nRGB: %u", (int)RGBvals[i]);
	}
	warpPrint("\n%u", (uint32_t)Rroll[rollNumber]);
	warpPrint("\n%u", (uint32_t)Groll[rollNumber]);
	warpPrint("\n%u", (uint32_t)Broll[rollNumber]);
	#endif
	*/
	for(int i = 0; i<2; i++){
		if(RGBvals[i] > 4096) RGBvals[i] = 4096;
	}
	if(RGBvals[0] > 60) { RGBvals[0] -= 60;} else {RGBvals[0] = 0;}
	if(RGBvals[1] > 30) { RGBvals[1] -= 30;} else {RGBvals[1] = 0;}
	//warpPrint("\nG:\t%u\tB:\t%u", (int)RGBvals[0], (int)RGBvals[1]);
	PWM_SetDuty(pwm_B, RGBvals[0]);
	PWM_SetDuty(pwm_G, RGBvals[1]);
}

void SetBaseRGB(uint16_t * RGBvals){
	static uint16_t Rroll[rollNumber+1];
	if(RGBvals[0] > 60) { RGBvals[0] -= 60;} else {RGBvals[0] = 0;}
	//RGBvals[0] = (RGBvals[0] * RGBvals[0] ) / 2048;
	if(RGBvals[0] > 4096) RGBvals[0] = 4096;
	PWM_SetDuty(pwm_R, rollingAverage(RGBvals[0], &Rroll[0], &Ridx));
	PWM_SetDuty(pwm_R, RGBvals[0]);
	warpPrint("\nR:\t%u", (int)RGBvals[0]);
}

void printFFT(int complex * x, int n){
	static int j = 0;
	if(j%100 == 0){
		for(int i=0; i < n/2; i++){
			warpPrint("\nHz:%u\t%u",i*(594/4),(uint32_t)cabs(x[i])); 
		}
	}
	j++;
}

uint16_t impulseConstant[10] = {20,13,9,7,5,4,3,2};
uint16_t impulseNormal = 67;

uint16_t rollingAverage(uint16_t newVal, uint16_t * rollArray, uint8_t * idx){
	uint16_t oldVal = rollArray[*idx]; 
	rollArray[*idx] = newVal;
	//int total = (int)rollArray[rollNumber];
	//int toAdd = ((int)newVal - (int)oldVal)/rollNumber;
	//int newTotal = total + toAdd;
	//rollArray[rollNumber] += (uint16_t)newTotal;
	//*idx = (*idx+1) % rollNumber;
	uint16_t total = 0;
	for(int i = 0; i<rollNumber; i++){
		total += (rollArray[(*idx + i) % rollNumber] * impulseConstant[i] )/ impulseNormal;
	}
	return total;
}
