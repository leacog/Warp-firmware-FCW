#include "PWM.h"
#include "fsl_tpm_driver.h"
#include "fsl_tpm_hal.h"

const tpm_general_config_t basePwmConfig = {
  .isBDGMode = false,
  .isGlobalTimeBase = false,
  .isTriggerMode = false,
  .isStopCountOnOverflow = false,
  .isCountReloadOnTrig = false
};

const tpm_pwm_param_t pwmConfig = {
  .mode = kTpmEdgeAlignedPWM,
  .edgeMode = kTpmHighTrue,
  .uFrequencyHZ = 1,
  .uDutyCyclePercent = 50
};


void PWM_init(uint8_t instance, uint8_t channel){
  TPM_DRV_Init(instance, &basePwmConfig);
  TPM_DRV_Init(instance, kTpmClockSourceModuleHighFreq, kTpmDividedBy128); //Set prescaler to 128 - clock is now 375kHz
  TPM_DRV_PwmStart(instance, &pwmConfig, channel);
}
