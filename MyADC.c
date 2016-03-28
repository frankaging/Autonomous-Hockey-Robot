/*
* MyADC.c
*
* Created: 11/6/2015 1:24:51 PM
*  Author: xuzhi
*/

#include"m_general.h"

void init_ADC(){
	m_clockdivide(0);

	clear(DDRF,0);
	clear(DDRF,1);
	clear(DDRF,4);
	clear(DDRF,5);
	clear(DDRF,6);
	clear(DDRF,7);
	clear(DDRD,4);

	clear(ADMUX,REFS1);//set to Vcc
	set(ADMUX, REFS0);

	set(ADCSRA, ADPS2);//set ADC prescaler to /128
	set(ADCSRA, ADPS1);
	set(ADCSRA, ADPS0);

	set(DIDR0, ADC0D);//disable digital inputs F0
	set(DIDR0, ADC1D);//disable digital inputs F1
	set(DIDR0, ADC4D);//disable digital inputs F4
	set(DIDR0, ADC5D);//disable digital inputs F5
	set(DIDR0, ADC6D);//disable digital inputs F6
	set(DIDR0, ADC7D);//disable digital inputs F7
	set(DIDR2 , ADC8D);//disable digital inputs D4
	set(ADCSRA, ADATE);//free running
}

void ADCConversion(int* data){
	// F0
	clear(ADCSRA, ADEN);//disable ADC system                                                //Left PID Puck Control Photo-transistor

	clear(ADCSRB, MUX5);//channel F0
	clear(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	clear(ADMUX, MUX0);

	set(ADCSRA, ADEN);//enable ADC system
	set(ADCSRA, ADSC);//Start Conversion

	while(check(ADCSRA, ADIF) == 0){}
	data[0] = ADC;//read F0
	clear(ADCSRA, ADEN);//disable ADC system

	//F1
	clear(ADCSRB, MUX5);//channel F1                                                                //Right PID Puck Control Photo-transistor
	clear(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	set(ADMUX, MUX0);

	clear(ADCSRA, ADIF);//demask flag
	set(ADCSRA, ADEN);//enable ADC system
	set(ADCSRA, ADSC);//Start Conversion

	while(check(ADCSRA, ADIF) == 0){}
	data[1] = ADC;//read F1
	clear(ADCSRA, ADEN);//disable ADC system

	//F4                                                                                                                    //Puck Containing Control
	clear(ADCSRB, MUX5);//channel F4
	set(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	clear(ADMUX, MUX0);

	clear(ADCSRA, ADIF);//demask flag
	set(ADCSRA, ADEN);//enable ADC system
	set(ADCSRA, ADSC);//Start Conversion

	while(check(ADCSRA, ADIF) == 0){}
	data[2] = ADC;//read F4
	clear(ADCSRA, ADEN);//disable ADC system

	//F5
	clear(ADCSRB, MUX5);//channel F5 //                                                             //Left Front Direction Control
	set(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	set(ADMUX, MUX0);

	clear(ADCSRA, ADIF);//demask flag
	set(ADCSRA, ADEN);//enable ADC system
	set(ADCSRA, ADSC);//Start Conversion

	while(check(ADCSRA, ADIF) == 0){}
	data[3] = ADC;//read F5 //
	clear(ADCSRA, ADEN);//disable ADC system

	//F6
	clear(ADCSRB, MUX5);//channel F6                                                                //Right Front Direction Control
	set(ADMUX, MUX2);
	set(ADMUX, MUX1);
	clear(ADMUX, MUX0);

	clear(ADCSRA, ADIF);//demask flag
	set(ADCSRA, ADEN);//enable ADC system
	set(ADCSRA, ADSC);//Start Conversion

	while(check(ADCSRA, ADIF) == 0){}
	data[4] = ADC;//read F6
	clear(ADCSRA, ADEN);//disable ADC system

	//F7
	clear(ADCSRB, MUX5);//channel F7                                                                //Left Back Direction Control
	set(ADMUX, MUX2);
	set(ADMUX, MUX1);
	set(ADMUX, MUX0);

	clear(ADCSRA, ADIF);//demask flag
	set(ADCSRA, ADEN);//enable ADC system
	set(ADCSRA, ADSC);//Start Conversion

	while(check(ADCSRA, ADIF) == 0){}
	data[5] = ADC;//read F7
	clear(ADCSRA, ADEN);//disable ADC system

	//D4
	set(ADCSRB, MUX5);//channel D4                                                                  //Right Back Direction Control
	clear(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	clear(ADMUX, MUX0);

	clear(ADCSRA, ADIF);//demask flag
	set(ADCSRA, ADEN);//enable ADC system
	set(ADCSRA, ADSC);//Start Conversion

	while(check(ADCSRA, ADIF) == 0){}
	data[6] = ADC;//read D4
	clear(ADCSRA, ADEN);//disable ADC system
}

//lightL: ADC value of left phototransistor;
//lightR: ADC value of right phototransistor;
//omega: angular velocity;
//PD: parameter needed to be returned;
void puck(int lightL, int lightR, float omega,  int* PD){
	int kp = 1000;
	int kd = 0;
	int diffLight = lightL - lightR;
	*PD = kp * diffLight + kd * omega;
}

//return true when lightPuck is lower than 512
bool getPuck(int lightPuck){
	if(lightPuck>=512){
		return false;
		}else{
		return true;
	}
}