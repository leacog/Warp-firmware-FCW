#include "FFT.h"
#include "complex.h"
#define PRE_MULT 1000

static long complex tf64[32] = { 
1000     + -0    *I,
995      + -98   *I,
981      + -195  *I,
957      + -290  *I,
924      + -383  *I,
882      + -471  *I,
831      + -556  *I,
773      + -634  *I,
707      + -707  *I,
634      + -773  *I,
556      + -831  *I,
471      + -882  *I,
383      + -924  *I,
290      + -957  *I,
195      + -981  *I,
98       + -995  *I,
0        + -1000 *I,
-98      + -995  *I,
-195     + -981  *I,
-290     + -957  *I,
-383     + -924  *I,
-471     + -882  *I,
-556     + -831  *I,
-634     + -773  *I,
-707     + -707  *I,
-773     + -634  *I,
-831     + -556  *I,
-882     + -471  *I,
-924     + -383  *I,
-957     + -290  *I,
-981     + -195  *I,
-995     + -98   *I
};

static int BitReverseArray[64] = {0,32,16,48,8,40,24,56,4,36,20,52,12,44,28,60,2,34,18,50,10,42,26,58,6,38,22,54,14,46,30,62,1,33,17,49,9,41,25,57,5,37,21,53,13,45,29,61,3,35,19,51,11,43,27,59,7,39,23,55,15,47,31,63};

void FFT(long complex * x, int N){

  //long complex x_ping[N];
  
  bitReverse(x, N);

  FFTN(&x[0], N);
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
  
  */
}

void bitReverse(long complex * x, int N){
  long complex x_temp[N];
  for(int i =0; i < N; i++){
    x_temp[i] = x[BitReverseArray[i*(64/N)]]; 
  }
  for(int i =0; i < N; i++){
    x[i] = x_temp[i]; 
  }
}


/*

void FFT2(long complex * x2, long complex * outArray){
  outArray[0] = x2[0] + x2[1];
  outArray[1] = x2[0] - x2[1];
}

void FFT4(long complex * x4, long complex * outArray){
  for(int i = 0; i<2; i++){
	  outArray[i] = (1000*x4[i] + tf64[i*(32/4)] * x4[i+2])/1000;
	  outArray[i+2] = (1000*x4[i] - tf64[i*(32/4)] * x4[i+2])/1000;
  }
}

void FFT8(long complex * x8, long complex * outArray){
  for(int i =0; i<4; i++){
  	outArray[i]   = (1000 * x8[i] + tf64[i*(32/8)] * x8[i+4]);
	outArray[i+4] = (1000 * x8[i] - tf64[i*(32/8)] * x8[i+4]);
  }
}

void FFT16(long complex * x16, long complex * outArray){
  for(int i =0; i<8; i++){
  	outArray[i]   = (1000 * x16[i] + tf64[i*(32/16)] * x16[i+8]);
	outArray[i+8] = (1000 * x16[i] - tf64[i*(32/16)] * x16[i+8]);
  }
}
*/

//
void FFTN(long complex * x, int n){
  if(n>2){
    FFTN(&x[0],   n/2);
    FFTN(&x[n/2], n/2);
  }
  for(int i=0; i < (n/2); i++){
    long complex x_temp_1 = x[i];
    long complex x_temp_2 = x[i+n/2];    
    x[i]     = (PRE_MULT * x_temp_1 + tf64[i*(64/n)] * x_temp_2)/PRE_MULT;
    x[i+n/2] = (PRE_MULT * x_temp_1 - tf64[i*(64/n)] * x_temp_2)/PRE_MULT;
  }
}
