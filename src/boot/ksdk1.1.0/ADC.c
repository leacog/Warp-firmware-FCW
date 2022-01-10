#include "ADC.h"
#include "fsl_adc16_driver.h"

void ADC16_init_continuous(uint32_t instance, uint32_t chnGroup, uint8_t chn)
{
	adc16_user_config_t MyAdcUserConfig;
	adc16_chn_config_t MyChnConfig;

	// Initialize the ADC converter. //
	ADC16_DRV_StructInitUserConfigDefault(&MyAdcUserConfig);
	MyAdcUserConfig.continuousConvEnable = true; // Enable continuous conversion. //
	MyAdcUserConfig.clkSrcMode = kAdcClkSrcOfBusOrAltClk2; //Set bus clock divided by two
	MyAdcUserConfig.clkDividerMode = kAdcClkDividerInputOf2;
	MyAdcUserConfig.intEnable = true;
	ADC16_DRV_Init(instance, &MyAdcUserConfig);
	// Configure the ADC channel and take an initial trigger. //
	MyChnConfig.chnNum = chn;
	MyChnConfig.diffEnable= false;
	MyChnConfig.intEnable = true;
	//MyChnConfig.chnMux = kAdcChnMuxOfA; // No MUX mode for our MCU
	adc16_hw_average_count_mode_t mode = kAdcHwAverageCountOf16;
	ADC16_DRV_EnableHwAverage(instance, mode);
	ADC16_DRV_ConfigConvChn(instance, chnGroup, &MyChnConfig);
};
