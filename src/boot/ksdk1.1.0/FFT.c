#include "FFT.h"
#include "complex.h"
#define PRE_MULT 1000

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


void FFT(long complex * eightPoints, long complex * outArray, int N){
  static int BitReverseArray[32] = {0,16,8,24,4,20,12,28,2,18,10,26,6,22,14,30,1,17,9,25,5,21,13,29,3,19,11,27,7,23,15,31};

  //long complex x_ping[N];
  long complex x_temp[N];
  
  for(int i =0; i < N; i++){
    x_temp[i] = eightPoints[BitReverseArray[i*(32/N)]]; 
  }
  
  FFTN(&x_temp[0], &outArray[0], N);
  /*
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
  */
}

/*

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
*/

void FFTN(long complex * x, long complex * output, int n){
  warpPrint("\nn=%u", n);
  if(n>2){
    FFTN(&x[0],   &x[0],   n/2);
    FFTN(&x[n/2], &x[n/2], n/2);
  }
  for(int i=0; i < (n/2); i++){
    long complex x_temp_1 = x[i];
    long complex x_temp_2 = x[i+n/2];    
    output[i]     = (PRE_MULT * x_temp_1 + tf32[i*(32/n)] * x_temp_2)/PRE_MULT;
    output[i+n/2] = (PRE_MULT * x_temp_1 - tf32[i*(32/n)] * x_temp_2)/PRE_MULT;
  }
}
