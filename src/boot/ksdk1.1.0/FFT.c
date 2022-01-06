#include "FFT.h"

void FFT(uint32_t * eightPoints, uint32_t * outArray){
  static int BitReverseArray[8] = {0,4,2,6,1,5,3,7};

  uint32_t x_ping[8];
  uint32_t x_pong[8];
  memcpy(&x_ping[0], eightPoints, 8);

  for(int i =0; i < 8; i++){
    x_pong[i] = x_ping[BitReverseArray[i]]; 
  }
  
  for(int i=0; i < 4; i++){
    FFT2(x_pong[i*2], x_ping[i*2]);
  }
  memcpy(outArray, &x_ping[0], 8);
}

void FFT2(uint32_t * twoPoints, uint32_t * outArray){
  static int BitReverseArray[2] = {0, 1};
  uint32_t x_rev[2];
  uint32_t x[2];
  memcpy(&x_rev[0], twoPoints, 2);

  for(int i =0; i < 2; i++){
    x[i] = x_rev[BitReverseArray[i]]; 
  }
  outArray[0] = x[0] + x[1];
  outArray[1] = x[0] - x[1];
}

/*
void FFT4(uint32_t * fourPoints, uint32_t * outArray){
  static int BitReverseArray[4] = {0, 1};
  uint32_t x_rev[2];
  uint32_t x[2];
  memcpy(&x_rev[0], twoPoints, 2);

  for(int i =0; i < 2; i++){
    x[i] = x_rev[BitReverseArray[i]]; 
  }
  outArray[0] = x[0] + x[1];
  outArray[1] = x[0] - x[1];
}
*/