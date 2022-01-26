#include "complex.h"
#include "stdint.h"

void FFT(int complex * x, uint8_t N);
void FFTN(int complex * x, uint8_t n);
void bitReverse(uint16_t * x, uint8_t N);
void applyWindow32(int * samples);
void octaves(int complex * x, uint16_t * output, uint8_t N);
