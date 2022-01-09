#include "ADC.h"
#include "fsl_adc16_driver.h"

void ADC16_init_continuous(uint32_t instance, uint32_t chnGroup, uint8_t chn)
{

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
	adc16_calibration_param_t MyAdcCalibraitionParam;
#endif // FSL_FEATURE_ADC16_HAS_CALIBRATION //

	adc16_user_config_t MyAdcUserConfig;
	adc16_chn_config_t MyChnConfig;
	uint16_t MyAdcValue;

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
	// Auto calibraion. //
	ADC16_DRV_GetAutoCalibrationParam(instance, &MyAdcCalibraitionParam);
	ADC16_DRV_SetCalibrationParam(instance, &MyAdcCalibraitionParam);
#endif // FSL_FEATURE_ADC16_HAS_CALIBRATION //

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

uint32_t ADC16_poll_blocking(uint32_t instance, uint32_t chnGroup){
	ADC16_DRV_WaitConvDone(instance, chnGroup);
	uint16_t adcValue = ADC16_DRV_GetConvValueRAW(instance, chnGroup);
	return 0;
	//return ADC16_DRV_ConvRAWData(adcValue, false, kAdcResolutionBitOfSingleEndAs12); 
}
