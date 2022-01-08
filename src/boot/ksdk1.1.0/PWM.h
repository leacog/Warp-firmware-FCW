#include "stdint.h"

void PWM_init(uint8_t instance, uint8_t channel);
void PWM_SetDuty(uint8_t instance, uint8_t chn, uint16_t val);
void TPM_init(uint8_t instance);
