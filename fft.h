/*
 * fft.h
 *
 * Created on: 15 mai 2021
 * Author: Imported from TP5
 *
 * Make computation for AudirProcessing
 */
#ifndef FFT_H
#define FFT_H


typedef struct complex_float{
	float real;
	float imag;
}complex_float;

void doFFT_optimized(uint16_t size, float* complex_buffer);

void doFFT_c(uint16_t size, complex_float* complex_buffer);

#endif /* FFT_H */
