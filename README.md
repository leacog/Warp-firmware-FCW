# LitShow Shield
Mikael Cognell - __mec77__ 

4B25: Coursework Item 5 

St Catharine's College 

<img src="images/4B25_LitShield.jpg" width="60%">

## System Overview
The LitShow shield runs on a Freescale/NXP FRDM KL03 evaluation board and  creates a real-time light show to music. The board uses a microphone to measure audio, analyses the signal on the MCU and outputs three channels of Pulse-Width Modulated 12V for LED strips. In the current firmware, the channels each respond to a different frequency band.

<img src="images/4B25_LS_Overview.png" width="80%" align="center"/> 

## LitShow Shield
#### Microphone
The shield provides two connectors for microphones and expects a 0-3.3V(max) pre-amplied signal with a DC offset at 1/2 VCC (DC offset is adjustable in software). The 3-pin connector matches the pinout of many commonly avaialable electret microphone breakout boards - such as the [adafruit MAX4477](https://www.adafruit.com/product/1063). The 5-pin connector matches the pinout of the adafruit breakoutboard for the [MAX9814](https://www.adafruit.com/product/1713) with Automatic Gain Control.

The output from the microphone is interfaced to the MCU via a RC low-pass filter, with a -3dB frequncy of 20kHz. Furhter work on filtering could be useful, as the microphone still picks up some noise from the serial communication.
#### 12V Output Stage
Three BCP56T1G NPN transistors in a common emitter configuration amplify the MCU PWM output. The 12V rail for the LEDs is supplied by an external power supply connected via a barrel jack at the edge of the shield. Addmittedly MOSFETs would make more sense in this situation, however these were the only components I had on hand.

#### Hardware files
The KiCad PCB project files are included in the repo. Note that they are designed for a single sided board, as I'm etching my PCB at home using a positive photoresist method. If ordering boards from a fab, it's worth considering rerouting the board to be double sided as certain headers (e.g. microphone header) currently have to be soldered on the wrong side.
## FRDM-KL03Z Firmware
The shield connects to a Freescale/NXP FRDM KL03 evaluation board, which performs the signal processing of the audio and PWM generation. The software is based on the Warp-firmware, but most of the facilities have been removed and only some of the initialization (clock & power management) remain. 

The software is split into three components/source files:
#### [ADC.c](src\boot\ksdk1.1.0\ADC.c) - Analog to digtal conversion
The ADC is configured to sample the audio at 38kHz (to fufill the nyquist sampling criteria for the audible range) and stores the result in a buffer. To ensure a constant delay between samples, the ADC operates in continuous conversion mode. At the completion of a conversion (measurment), an interupt signal is generated and the ADC0_IRQHandler() ISR stores the latest sample in a buffer. Using interupts as opposed to e.g. a polling approach allows the main FFT algorithm to run while the next batch of samples are collected.

#### [FFT.c](src\boot\ksdk1.1.0\FFT.c) - Fast Fourier Transform
To extract freqeuncy information a discret fourier transform is computed on the sample batch using a Radix-2 FFT algorithm. There are two compilation options: recursive N-point and 64-point hardcoded.

The MKL03Z32VFK4 MCU is very constrained in memory, with only 2KB of SRAM. To minimize memory usage, the FFT algorithm is implemented in-place i.e. operates directly on the complex input array. To minimize the peak demand on the stack, the sample re-ordering via bit-reversal which is required at the start of the FFT is executed before the complex array is put on the stack. This way the two arrays never co-exist on the stack. The result is that the algorithm only occupies N * sizeof(int complex) + 2 of memory on the stack (514 Bytes for 64 point FFT). Still, to accomodate the high memory demand, the default Warp linker script is modified to increase maximum size of the stack, and several modifications to the Warp firmware was performed to minimize the RAM usage (which leaves more space for the stack).

For increased performance, the [twiddle factors](https://en.wikipedia.org/wiki/Twiddle\_factor) are precomputed. This avoids any computation of trigonometric quantities during the course of the program. In a similar manner (although much less critical), the bit reversal is also implemented in memory rather than by bit operations. To generate the twiddle factors and bitReverse arrays, a python command line script is provided under pythonScripts.

The twiddle factors - which are multiplied with sample values - lie between -1 and 1. As the MCU lacks a Floating Point Unit, the FFT is done with fixed point arithmetic. Since the default complex type doesn't support bit-shifting 

The FFT.c file also provides a helper routine for splitting the power spectrum into octaves, which is better aligned with the human logarithic perception of frequency. This is used by the PWM routines to determine brightness values.

#### [PWM.c](src\boot\ksdk1.1.0\PWM.c) - Mapping frequency information to duty cycles
The PWM.c functionality sets up the Timer modules of the MCU to output PWM signals on PTA5, PTA6 and PTB6. It contains a function to update the duty cycles for each of the channels, which accepts as arguments a costum pwmColour type which maps the respective colour to the corresponding timer instances and channels.

Each time an FFT completes in the main loop, the power spectrum is divided into three bins using the octaves function. The value of each bin then sets the duty cycle of a corresponding colour of LED. This means one LED channel responds to base tones, one to mid-range and one to trebble. There is a lot of scope for increasing the sophistication of the mapping from DFT to brightness values, as the current firmware basically only implements a low-pass, band-pass and high-pass filter (which could be done with 4 capacitors and 4 resistors...).

It also contains some functions to perform further processing of the frequency signals. For the base note, a first-order lag is applied to make the brightness linger more, as this produces a nice effect. 