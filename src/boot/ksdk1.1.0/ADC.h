#include "fsl_adc16_driver.h"
void ADC16_init_continuous(uint32_t instance, uint32_t chnGroup, uint8_t chn);
uint32_t ADC16_poll_blocking(uint32_t instance, uint32_t chnGroup);
