#include "FFT.h"
#include "string.h" //Needed for memcpy
#include "complex.h"

void FFT(long complex * eightPoints, long complex * outArray){
  static int BitReverseArray[8] = {0,4,2,6,1,5,3,7};

  long complex x_ping[8];
  long complex x_pong[8];

  for(int i =0; i < 8; i++){
    x_pong[i] = eightPoints[BitReverseArray[i]]; 
  }
  
  for(int i=0; i < 4; i++){
    FFT2(&x_pong[i*2], &x_ping[i*2]);
  }
  
  for(int i=0; i < 2; i++){
    FFT4(&x_ping[i*4], &x_pong[i*4]);
  }
  
  for(int i =0; i<8; i++){
  	outArray[i] = x_ping[i];
  }
}

void FFT2(long complex * twoPoints, long complex * outArray){
  outArray[0] = twoPoints[0] + twoPoints[1];
  outArray[1] = twoPoints[0] - twoPoints[1];
}

void FFT4(long complex * fourPoints, long complex * outArray){
  long complex twiddleFactors [4] = { 1, 0-1*I, -1, 0+1*I};
  outArray[0] = x[0] + x[2];
  outArray[1] = x[1] + twiddleFactors[1] * x[1];
  outArray[2] = x[0] - x[2];
  outArray[3] = x[1] + twiddleFactors[4] * x[3];
}

/*
void FFT4(long complex * fourPoints, long complex * outArray){
  long complex twiddleFactors [4] = { 1, 0-1*I, -1, 0+1*I};
  outArray[0] = x[0] + x[2];
  outArray[1] = x[1] + twiddleFactors[1] * x[1];
  outArray[2] = x[0] - x[2];
  outArray[3] = x[1] + twiddleFactors[4] * x[3];
}
*/


