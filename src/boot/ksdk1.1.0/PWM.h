#include "stdint.h"

typedef struct pwmColour {
	uint8_t tpmInstance;
	uint8_t tpmChannel;
} pwmColour;
	
pwmColour pwm_R = {.tpmInstance = 0, .tpmChannel =0};
pwmColour pwm_B = {.tpmInstance = 0, .tpmChannel =1};
pwmColour pwm_G = {.tpmInstance = 1, .tpmChannel =1};


void PWM_init(pwmColour colour);
void PWM_SetDuty(pwmColour colour, uint16_t val);
void TPM_init(uint8_t instance);
