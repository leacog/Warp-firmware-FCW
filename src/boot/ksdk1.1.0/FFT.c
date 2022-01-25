#include "FFT.h"
#include "complex.h"
#define PRE_MULT 1000

// static int complex tf64[32] = {  //Static ensures array is not stored in RAM
// 1000     + -0    *I,
// 995      + -98   *I,
// 981      + -195  *I,
// 957      + -290  *I,
// 924      + -383  *I,
// 882      + -471  *I,
// 831      + -556  *I,
// 773      + -634  *I,
// 707      + -707  *I,
// 634      + -773  *I,
// 556      + -831  *I,
// 471      + -882  *I,
// 383      + -924  *I,
// 290      + -957  *I,
// 195      + -981  *I,
// 98       + -995  *I,
// 0        + -1000 *I,
// -98      + -995  *I,
// -195     + -981  *I,
// -290     + -957  *I,
// -383     + -924  *I,
// -471     + -882  *I,
// -556     + -831  *I,
// -634     + -773  *I,
// -707     + -707  *I,
// -773     + -634  *I,
// -831     + -556  *I,
// -882     + -471  *I,
// -924     + -383  *I,
// -957     + -290  *I,
// -981     + -195  *I,
// -995     + -98   *I
// };

static cNumber tf64[32] = {
{.real = 32767   , .imag =-0},
{.real = 32610   , .imag =-3212},
{.real = 32138   , .imag =-6393},
{.real = 31357   , .imag =-9512},
{.real = 30274   , .imag =-12540},
{.real = 28899   , .imag =-15447},
{.real = 27246   , .imag =-18205},
{.real = 25330   , .imag =-20788},
{.real = 23170   , .imag =-23170},
{.real = 20788   , .imag =-25330},
{.real = 18205   , .imag =-27246},
{.real = 15447   , .imag =-28899},
{.real = 12540   , .imag =-30274},
{.real = 9512    , .imag =-31357},
{.real = 6393    , .imag =-32138},
{.real = 3212    , .imag =-32610},
{.real = 0       , .imag =-32767},
{.real = -3212   , .imag =-32610},
{.real = -6393   , .imag =-32138},
{.real = -9512   , .imag =-31357},
{.real = -12540  , .imag =-30274},
{.real = -15447  , .imag =-28899},
{.real = -18205  , .imag =-27246},
{.real = -20788  , .imag =-25330},
{.real = -23170  , .imag =-23170},
{.real = -25330  , .imag =-20788},
{.real = -27246  , .imag =-18205},
{.real = -28899  , .imag =-15447},
{.real = -30274  , .imag =-12540},
{.real = -31357  , .imag =-9512},
{.real = -32138  , .imag =-6393},
{.real = -32610  , .imag =-3212},
};

static int BitReverseArray[64] = {0,32,16,48,8,40,24,56,4,36,20,52,12,44,28,60,2,34,18,50,10,42,26,58,6,38,22,54,14,46,30,62,1,33,17,49,9,41,25,57,5,37,21,53,13,45,29,61,3,35,19,51,11,43,27,59,7,39,23,55,15,47,31,63};
static int hanning32[32] = {0,83,331,734,1273,1929,2673,3475,4303,5122,5899,6603,7203,7677,8004,8170,8170,8004,7677,7203,6603,5899,5122,4303,3475,2673,1929,1273,734,331,83,0};

void FFT(cNumber * x, uint8_t N){
  FFTN(&x[0], N);
}

void applyWindow32(int * samples){
  for(int i = 0; i < 32; i++){
    samples[i] = (samples[i] * hanning32[i]) / 8192; //Using 2^13 = 8192 for hanning factors -> could do shift if using unsigned or some trickery if faster is needed
  }
}

void bitReverse(int * x, uint8_t N){
  int x_temp[N];
  for(uint8_t i =0; i < N; i++){
    x_temp[i] = x[BitReverseArray[i*(64/N)]]; 
  }
  for(uint8_t i =0; i < N; i++){
    x[i] = x_temp[i]; 
  }
}

void octaves(cNumber * x, uint16_t * output, uint8_t N){
/* General solution is difficult to tweek but keep here for reference  
  uint8_t log2N = 0;
  //Can't get octaves lower than for N=8 -> now maps to 8->0, 16->1 32->2 etc.
  N >>= 4;
  while(N > 0){
	  log2N++;
	  N >>= 1;
  }
  
  uint8_t idx = 0;
  for(int i = 0; i < 3; i++){
	uint8_t j = (1u<<log2N) << i;     //i=0, N=8 -> j = 1      i=0, N=16 -> j = 2 etc... i=1, N = 16 -> j = 4, i=2, N=16 -> j = 8.
  	for(uint8_t k = 0; k < j; k++){
		output[i] += cabs(x[idx]);
		idx++;
	}
  	output[i] = (output[i] >> log2N) >> i;
  } 
*/
  for(uint8_t i = 1; i<=2; i++)  { output[0] += x[i].real * x[i].real + x[i].imag * x[i].imag; }
  output[0] >>= 1;
  for(uint8_t i = 5; i<=8; i++)  { output[1] += x[i].real * x[i].real + x[i].imag * x[i].imag; }
  output[1] >>= 1;
  for(uint8_t i = 16; i<=28; i++){ output[2] += x[i].real * x[i].real + x[i].imag * x[i].imag; }
  output[2] >>= 2;
}

void FFTN(cNumber * x, uint8_t n){
  if(n>2){
    FFTN(&x[0],   n/2);
    FFTN(&x[n/2], n/2);
  }

  for(uint8_t i=0; i < (n/2); i++){
    int x_temp_1_im = 0;
    int x_temp_1_re = 0;
    memcpy(&x_temp_1_im, &(x[i].imag), 4); //Copies x[i] to top 16 bits of x_temp_1
    memcpy(&x_temp_1_re, &(x[i].real), 4); 
    
    int x_temp_2_im = (int)(x[i+n/2].imag) * (int)(tf64[i*(64/n)].real) + (int)(x[i+n/2].real) * (int)(tf64[i*(64/n)].imag); 
    int x_temp_2_re = (int)(x[i+n/2].real) * (int)(tf64[i*(64/n)].real) - (int)(x[i+n/2].imag) * (int)(tf64[i*(64/n)].imag);
    int x_temp_2_im *= 2;    //Because of the sign-bit, only shift by 15 bits, so need to multiply by 2 again.
    int x_temp_2_re *= 2;    

    int x_temp_i_im = x_temp_1_im + x_temp_2_im;
    int x_temp_i_re = x_temp_1_re + x_temp_2_re;
    int x_temp_in2_im = x_temp_1_im - x_temp_2_im;
    int x_temp_in2_re = x_temp_1_re - x_temp_2_re;

    memcpy(&(x[i].imag), &x_temp_i_im, 4);              //Copies top 16 bits, effectivly dividing by 2^16     
    memcpy(&(x[i].real), &x_temp_i_re, 4);
    memcpy(&(x[i + n/2].imag), &x_temp_in2_im, 4);
    memcpy(&(x[i + n/2].real), &x_temp_in2_re, 4);
  }
}

/*
void FFTN(int complex * x, uint8_t n){
  if(n>2){
    FFTN(&x[0],   n/2);
    FFTN(&x[n/2], n/2);
  }
  for(uint8_t i=0; i < (n/2); i++){
    int complex x_temp_1 = x[i];
    int complex x_temp_2 = x[i+n/2];    
    x[i]     = (PRE_MULT * x_temp_1 + tf64[i*(64/n)] * x_temp_2)/PRE_MULT;
    x[i+n/2] = (PRE_MULT * x_temp_1 - tf64[i*(64/n)] * x_temp_2)/PRE_MULT;
  }
}
*/