#include "complex.h"
#include "stdint.h"

typedef struct cNumber{
  int16_t real;
  int16_t imag;
} cNumber;

void FFT(cNumber * x, uint8_t N);
void FFTN(cNumber * x, uint8_t n);
void bitReverse(volatile uint16_t * x, uint8_t N);
void applyWindow32(int * samples);
void octaves(cNumber * x, uint16_t * output, uint8_t N);

