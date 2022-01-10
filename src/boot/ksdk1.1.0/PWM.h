#include "stdint.h"

typedef struct pwmColour {
	uint8_t tpmInstance;
	uint8_t tpmChannel;
} pwmColour;
	
pwmColour pwm_R = {.tpmInstance = 0, .tpmChannel =0}; //PTA6
pwmColour pwm_G = {.tpmInstance = 0, .tpmChannel =1}; //PTA5
pwmColour pwm_B = {.tpmInstance = 1, .tpmChannel =1}; //PTB6


void PWM_init(pwmColour colour);
void PWM_SetDuty(pwmColour colour, uint16_t val);
void TPM_init(uint8_t instance);
