#include "FFT.h"
#include "complex.h"
#define PRE_MULT 1000

static int complex tf64[32] = {  //Static ensures array is not stored in RAM
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
static int hanning32[32] = {0,83,331,734,1273,1929,2673,3475,4303,5122,5899,6603,7203,7677,8004,8170,8170,8004,7677,7203,6603,5899,5122,4303,3475,2673,1929,1273,734,331,83,0};

void FFT(int complex * x, int N){
  bitReverse(x, N);
  FFTN(&x[0], N);
}

void applyWindow32(int * samples){
  for(int i = 0; i < 32; i++){
    samples[i] = (samples[i] * hanning32[i]) / 8192; //Using 2^13 = 8192 for hanning factors -> could do shift if using unsigned or some trickery if faster is needed
  }
}

void bitReverse(int complex * x, int N){
  int complex x_temp[N];
  for(int i =0; i < N; i++){
    x_temp[i] = x[BitReverseArray[i*(64/N)]]; 
  }
  for(int i =0; i < N; i++){
    x[i] = x_temp[i]; 
  }
}

void octaves(int complex * x, uint16_t * output){
  output[0] = (uint16_t) (cabs(x[1])+cabs(x[2])); //Discard DC component
  output[1] = (uint16_t) (cabs(x[3]) + cabs(x[4]) + cabs(x[5]) + cabs(x[6]));
  output[2] = (uint16_t) (cabs(x[7])+cabs(x[8])+cabs(x[9])+cabs(x[10])+cabs(x[11])+cabs(x[12])+cabs(x[13])+cabs(x[14]));
  output[0] = output[0] >> 1;
  output[1] = output[1] >> 2;
  output[2] = output[2] >> 2;
}

void FFTN(int complex * x, int n){
  if(n>2){
    FFTN(&x[0],   n/2);
    FFTN(&x[n/2], n/2);
  }
  for(int i=0; i < (n/2); i++){
    int complex x_temp_1 = x[i];
    int complex x_temp_2 = x[i+n/2];    
    x[i]     = (PRE_MULT * x_temp_1 + tf64[i*(64/n)] * x_temp_2)/PRE_MULT;
    x[i+n/2] = (PRE_MULT * x_temp_1 - tf64[i*(64/n)] * x_temp_2)/PRE_MULT;
  }
}
