/*
 * MyADC.h
 *
 * Created: 11/6/2015 1:25:01 PM
 *  Author: xuzhi
 */ 


#ifndef MYADC_H_
#define MYADC_H_

extern void init_ADC();
extern void ADCConversion(int*);
extern void puck(int lightL, int lightR, float omega, int* PD);
extern bool getPuck(int lightPuck);
#endif /* MYADC_H_ */