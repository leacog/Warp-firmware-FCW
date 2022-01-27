#include "FFT.h"
#include "complex.h"
#include "string.h"

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
//static int hanning32[32] = {0,83,331,734,1273,1929,2673,3475,4303,5122,5899,6603,7203,7677,8004,8170,8170,8004,7677,7203,6603,5899,5122,4303,3475,2673,1929,1273,734,331,83,0};

void FFT(cNumber * x, uint8_t N){
  FFTN(&x[0], N);
}

void applyWindow32(int * samples){
  for(int i = 0; i < 32; i++){
    //samples[i] = (samples[i] * hanning32[i]) / 8192; //Using 2^13 = 8192 for hanning factors -> could do shift if using unsigned or some trickery if faster is needed
  }
}

void bitReverse(volatile uint16_t * x, uint8_t N){
  uint16_t x_temp[N];
  for(uint8_t i =0; i < N; i++){
    x_temp[i] = x[BitReverseArray[i*(64/N)]]; 
  }
  for(uint8_t i =0; i < N; i++){
    x[i] = x_temp[i]; 
  }
}

void octaves(cNumber * x, uint16_t * output, uint8_t N){

  for(uint8_t i = 1; i<=2; i++)  {
	 int complex cTemp = x[i].real + x[i].imag * I; 
	 output[0] += (uint16_t)cabs(cTemp);
  }

  for(uint8_t i = 7; i<=10; i++)  {
	 int complex cTemp = x[i].real + x[i].imag * I; 
	 output[1] += (uint16_t)cabs(cTemp); 
  }
  
  for(uint8_t i = 16; i<=28; i++){
	 int complex cTemp = x[i].real + x[i].imag * I; 
	 output[2] += (uint16_t)cabs(cTemp); 
  }

/* General solution is difficult to tweek and not super readable but keept here for reference  
  uint8_t log2N = 0;
  //Can't get octaves lower than for N=8 -> N now maps to 8->0, 16->1 32->2 etc.
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
}

void FFTN(cNumber * x, uint8_t n){
  if(n>2){
    FFTN(&x[0],   n/2);
    FFTN(&x[n/2], n/2);
  }

  for(uint8_t i=0; i < (n/2); i++){
    
    int x_temp_1_im, x_temp_1_re;
    int x_temp_2_im, x_temp_2_re;
    
    x_temp_1_im =  (int)x[i].imag; 
    x_temp_1_re =  (int)x[i].real; 
    
    x_temp_2_im =  (int)x[i+n/2].imag;
    x_temp_2_re =  (int)x[i+n/2].real;

    x_temp_1_im *= 32768;  //Compiles optimizes this operation to a shift, known from tests
    x_temp_1_re *= 32768;
	
    int tf_im = (int)tf64[i*(64/n)].imag;  //Casting takes some time but avoid us storing twiddle factors as 32 bit numbers - halving required memory
    int tf_re =	(int)tf64[i*(64/n)].real;
	
    int xtf_temp_2_im = x_temp_2_im * tf_re	+	x_temp_2_re * tf_im;  //tf factors contain a factor of 2^15 
    int xtf_temp_2_re = x_temp_2_re * tf_re	-	x_temp_2_im * tf_im; 
    
    int x_temp_i_im = x_temp_1_im + xtf_temp_2_im;
    int x_temp_i_re = x_temp_1_re + xtf_temp_2_re;
    int x_temp_in2_im = x_temp_1_im - xtf_temp_2_im;
    int x_temp_in2_re = x_temp_1_re - xtf_temp_2_re;

    x[i].imag 		= (int16_t)(x_temp_i_im 	/ 32768);              //Dividing by 2^15 to renormalize     
    x[i].real 		= (int16_t)(x_temp_i_re 	/ 32768);
    x[i+(n/2)].imag 	= (int16_t)(x_temp_in2_im 	/ 32768);
    x[i+(n/2)].real 	= (int16_t)(x_temp_in2_re 	/ 32768);
  
    if(n==32 || n==16){
    	x[i].imag 		/= 2;              //With pm 2024 as largest possible input, we could reach 32768 by stage 16, so need to normalize down to fit in 16 bits
    	x[i].real 		/= 2;
    	x[i+(n/2)].imag 	/= 2;
    	x[i+(n/2)].real 	/= 2;
    }
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
