#include "complex.h"

void FFT(long complex * eightPoints, long complex * outArray, uint8_t N);
void FFT2(long complex * twoPoints, long complex * outArray);
void FFT4(long complex * fourPoints, long complex * outArray);
void FFT8(long complex * fourPoints, long complex * outArray);
void FFT16(long complex * fourPoints, long complex * outArray);
void FFTN(long complex * x, long complex * output, uint8_t n);
