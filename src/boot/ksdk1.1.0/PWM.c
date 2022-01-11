#include "PWM.h"
#include "fsl_tpm_driver.h"
#include "fsl_tpm_hal.h"

tpm_general_config_t basePwmConfig = {
  .isDBGMode = false,
  .isGlobalTimeBase = false,
  .isTriggerMode = false,
  .isStopCountOnOveflow = false,
  .isCountReloadOnTrig = true
};

tpm_pwm_param_t pwmConfig = {
  .mode = kTpmEdgeAlignedPWM,
  .edgeMode = kTpmHighTrue,
  .uFrequencyHZ = 1000,
  .uDutyCyclePercent = 0
};

const uint16_t decayConstants[10] = {20,13,9,7,5,4,3,2};
const uint16_t decayNormal = 67;

void TPM_init(uint8_t instance){
  TPM_DRV_Init(instance, &basePwmConfig);
  TPM_DRV_SetClock(instance, kTpmClockSourceModuleHighFreq, kTpmDividedBy16); //Set prescaler to 128 - clock is now 375kHz
}

void PWM_init(pwmColour colour){
  uint8_t instance = colour.tpmInstance;
  uint8_t channel  = colour.tpmChannel;
  TPM_DRV_PwmStart(instance, &pwmConfig, channel);
  TPM_HAL_SetMod(g_tpmBaseAddr[instance], 4095); //12 bit colour value
}

void PWM_SetDuty(pwmColour colour, uint16_t val){
	if(val > (4096-256)){
		val = 4096;
	}
	TPM_HAL_SetChnCountVal(g_tpmBaseAddr[colour.tpmInstance], colour.tpmChannel, val);
}

void SetTrebbleRGB(uint16_t * RGBvals){
	for(int i = 0; i<2; i++){
		if(RGBvals[i] > 4096) RGBvals[i] = 4096;
	}
	if(RGBvals[0] > 60) { RGBvals[0] -= 60;} else {RGBvals[0] = 0;}   //Remove pink noise - upgrade includes routine to do this dynamically
	if(RGBvals[1] > 30) { RGBvals[1] -= 30;} else {RGBvals[1] = 0;}
	PWM_SetDuty(pwm_B, RGBvals[0]);
	PWM_SetDuty(pwm_G, RGBvals[1]);
	//warpPrint("\nG:\t%u\tB:\t%u", (int)RGBvals[0], (int)RGBvals[1]);
}

void SetBaseRGB(uint16_t * RGBvals){
	static uint16_t Rroll[rollNumber+1];
	static uint8_t Ridx = 0;
	if(RGBvals[0] > 60) { RGBvals[0] -= 60;} else {RGBvals[0] = 0;}
	if(RGBvals[0] > 4096) RGBvals[0] = 4096;
	PWM_SetDuty(pwm_R, rollingAverage(RGBvals[0], &Rroll[0], Ridx));
	PWM_SetDuty(pwm_R, RGBvals[0]);
	warpPrint("\nR:\t%u", (int)RGBvals[0]);
}

uint16_t decayFilter(uint16_t newVal, uint16_t * rollArray, uint8_t idx){
	uint16_t oldVal = rollArray[idx]; 
	rollArray[idx] = newVal;
	uint16_t total = 0;
	for(int i = 0; i<rollNumber; i++){
		total += (rollArray[(*idx + i) % rollNumber] * decayConstants[i] )/ decayNormal;
	}
	return total;
}

uint16_t rollingAverage(uint16_t newVal, uint16_t * rollArray, uint8_t idx){
	uint16_t oldVal = rollArray[idx]; 
	rollArray[idx] = newVal;
	int total = (int)rollArray[rollNumber];
	int toAdd = ((int)newVal - (int)oldVal)/rollNumber;
	int newTotal = total + toAdd;
	rollArray[rollNumber] += (uint16_t)newTotal;
	*idx = (*idx+1) % rollNumber;
	return newTotal;
}
