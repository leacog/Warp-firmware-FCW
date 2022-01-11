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
