#include "FFT.h"
#include "string.h" //Needed for memcpy

void FFT(long * eightPoints, long * outArray){
  static int BitReverseArray[8] = {0,4,2,6,1,5,3,7};

  long x_ping[8];
  long x_pong[8];
  //memcpy(&x_ping[0], eightPoints, 8);

  for(int i =0; i < 8; i++){
    x_pong[i] = eightPoints[BitReverseArray[i]]; 
  }
  
  for(int i=0; i < 4; i++){
    FFT2(&x_pong[i*2], &x_ping[i*2]);
  }
  //memcpy(outArray, &x_ping[0], 8);
  for(int i =0; i<8; i++){
  	outArray[i] = x_ping[i];
  }
}

void FFT2(long * twoPoints, long * outArray){
  static int BitReverseArray[2] = {0, 1};
  //long x_rev[2];
  long x[2];
  //memcpy(&x_rev[0], twoPoints, 2);

  for(int i =0; i < 2; i++){
    x[i] = twoPoints[BitReverseArray[i]]; 
  }
  outArray[0] = x[0] + x[1];
  outArray[1] = x[0] - x[1];
}

/*
void FFT4(long * fourPoints, long * outArray){
  static int BitReverseArray[4] = {0, 1};
  long x_rev[2];
  long x[2];
  memcpy(&x_rev[0], twoPoints, 2);

  for(int i =0; i < 2; i++){
    x[i] = x_rev[BitReverseArray[i]]; 
  }
  outArray[0] = x[0] + x[1];
  outArray[1] = x[0] - x[1];
}
*/
