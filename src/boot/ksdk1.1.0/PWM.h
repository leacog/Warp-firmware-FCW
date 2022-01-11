#include "stdint.h"
typedef struct pwmColour {
	uint8_t tpmInstance;
	uint8_t tpmChannel;
} pwmColour;
	
pwmColour pwm_R = {.tpmInstance = 0, .tpmChannel =0}; //PTA6
pwmColour pwm_G = {.tpmInstance = 0, .tpmChannel =1}; //PTA5
pwmColour pwm_B = {.tpmInstance = 1, .tpmChannel =1}; //PTB6

void TPM_init(uint8_t instance);
void PWM_init(pwmColour colour);
void PWM_SetDuty(pwmColour colour, uint16_t val);
void SetTrebbleRGB(uint16_t * RGBvals);
void SetBaseRGB(uint16_t * RGBvals);
uint16_t decayFilter(uint16_t newVal, uint16_t * rollArray, uint8_t idx);
uint16_t rollingAverage(uint16_t newVal, uint16_t * rollArray, uint8_t idx);