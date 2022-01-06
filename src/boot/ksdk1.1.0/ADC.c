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
	ADC16_DRV_Init(instance, &MyAdcUserConfig);
	// Configure the ADC channel and take an initial trigger. //
	MyChnConfig.chnNum = chn;
	MyChnConfig.diffEnable= false;
	MyChnConfig.intEnable = false;
	//MyChnConfig.chnMux = kAdcChnMuxOfA; // No MUX mode for our MCU
	ADC16_DRV_ConfigConvChn(instance, chnGroup, &MyChnConfig);
};

