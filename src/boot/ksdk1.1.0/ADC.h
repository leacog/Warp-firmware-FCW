#include "fsl_adc16_driver.h"

void ADC16_start(void);
void ADC16_pause(void);

#define ADC_instance 0
#define ADC_chnGroup 0
#define ADC_chn      2       //Sets ADC channel up to. PTA9 = 2, PB12 = 0, PT0 = 15

adc16_user_config_t adcConfig = {
  //Default config
  .lowPowerEnable = false,
  .resolutionMode = kAdcResolutionBitOf12or13,
  .asyncClkEnable = false,
  .highSpeedEnable = false,
  .hwTriggerEnable = false,
  .refVoltSrcMode = kAdcRefVoltSrcOfVref,
  // Custom configuration
  .continuousConvEnable = true, // Enable continuous conversion. //
  .clkSrcMode = kAdcClkSrcOfBusOrAltClk2, //Set bus clock divided by two
  .clkDividerMode = kAdcClkDividerInputOf2,
  .intEnable = true
};

adc16_chn_config_t adcChnConfig = {
	.chnNum = ADC_chn,
	.diffEnable= false,
  .intEnable = true
};
