#include "FFT.h"
#include "string.h" //Needed for memcpy
#include "complex.h"

long complex tf32[16] = {
  1000     + -0    *I,
  981      + -195  *I,
  924      + -383  *I,
  831      + -556  *I,
  707      + -707  *I,
  556      + -831  *I,
  383      + -924  *I,
  195      + -981  *I,
  0        + -1000 *I,
  -195     + -981  *I,
  -383     + -924  *I,
  -556     + -831  *I,
  -707     + -707  *I,
  -831     + -556  *I,
  -924     + -383  *I,
  -981     + -195  *I
};


void FFT(long complex * eightPoints, long complex * outArray){
  static int BitReverseArray[16] = {0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};

  long complex x_ping[16];
  long complex x_pong[16];

  for(int i =0; i < 16; i++){
    x_pong[i] = eightPoints[BitReverseArray[i]]; 
  }
  
  for(int i=0; i < 8; i++){
    	FFT2(&x_pong[i*2], &x_ping[i*2]);
  }
  
  for(int i=0; i < 4; i++){
   	FFT4(&x_ping[i*4], &x_pong[i*4]);
  }
  
  for(int i=0; i<2; i++){
	FFT8(&x_pong[i*8], &x_ping[i*8]);
  }

  for(int i=0; i<1; i++){
  	FFT16(&x_ping[i*16], &x_pong[i*16]);
  }

  for(int i =0; i<16; i++){
  	outArray[i] = x_pong[i];
  }
}

void FFT2(long complex * x2, long complex * outArray){
  outArray[0] = x2[0] + x2[1];
  outArray[1] = x2[0] - x2[1];
}

void FFT4(long complex * x4, long complex * outArray){
  for(int i = 0; i<2; i++){
	  outArray[i] = (1000*x4[i] + tf32[i*(32/4)] * x4[i+2])/1000;
	  outArray[i+2] = (1000*x4[i] - tf32[i*(32/4)] * x4[i+2])/1000;
  }
}

void FFT8(long complex * x8, long complex * outArray){
  for(int i =0; i<4; i++){
  	outArray[i]   = (1000 * x8[i] + tf32[i*(32/8)] * x8[i+4]);
	outArray[i+4] = (1000 * x8[i] - tf32[i*(32/8)] * x8[i+4]);
  }
}

void FFT16(long complex * x16, long complex * outArray){
  for(int i =0; i<8; i++){
  	outArray[i]   = (1000 * x16[i] + tf32[i*(32/16)] * x16[i+8]);
	outArray[i+8] = (1000 * x16[i] - tf32[i*(32/16)] * x16[i+8]);
  }
}

