#include "complex.h"

void FFT(int complex * x, int N);
void FFTN(int complex * x, int n);
void bitReverse(int complex * x, int N);
void applyWindow32(int * samples);
void octaves(int complex * x, uint16_t * output);