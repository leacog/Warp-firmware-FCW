#include "ADC.h"
#include "fsl_adc16_driver.h"

void ADC16_start()
{
	ADC16_DRV_Init(ADC_instance, &adcConfig);
	adc16_hw_average_count_mode_t mode = kAdcHwAverageCountOf16;
	ADC16_DRV_EnableHwAverage(ADC_instance, mode);
	ADC16_DRV_ConfigConvChn(ADC_instance, ADC_chnGroup, &adcChnConfig);
};

void ADC16_pause()
{
  ADC16_DRV_PauseConv(ADC_instance, ADC_chnGroup);
}

