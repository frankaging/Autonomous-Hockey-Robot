/*
 * main.c
 *
 * Created: 11/4/2015 9:44:33 PM
 *  Author: wuzhengx
 */

#define F_CPU 16000000

#include <avr/io.h>
#include <math.h>
#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_wii.h"
#include "MyADC.h"

volatile int ifFourPoint = 0;

//define the coordinate corresponding number
#define X 0
#define Y 1

//define different game statues
#define COMMTEST 1
#define PLAY 2
#define GOALR 3
#define GOALB 4
#define PAUSE 5
#define HALFTIME 6
#define GAMEOVER 7
volatile int STATUE;

//define goal

//right goal
#define xGoal1 -200
#define yGoal1 300

//left goal
#define xGoal2 800
#define yGoal2 300

//define the initialization for mWifi module
#define CHANNEL 1
#define RXADDRESS 0x20 //reading address first robot 18, 19, 20
#define TXADDRESS 0x01 //sending address
#define PACKET_LENGTH 10
char buffer[10] = {0,0,0,0,0,0,0,0,0,0};

unsigned int blobs[12];                                         //Location data-set
int phoData[7] = {0, 0, 0, 0, 0, 0, 0};         //Photo-transistor data-set {Left PID(0), Right PID(1), Puck Containing(2), ...

float midEdge = 88; //88
float upperLeft = 48; //51
float upperRight = 40; //41
float lowerLeft = 60; //61
float lowerRight = 78; //78
float horiEdge = 70; //69
float floatDis = 3.9; //5
int stars[4][2];

void checkStatus(void);
//math function
int getDist(int a[], int b[]);
//main function
void getLocation(void);

void gettingPuck();
void leftMotorIni(void); //OCR1A & OCR1B
void RightMotorIni(void); //OCR1A & OCR1C
void MotorIni(void);
void sampleTimer(void);
void MyPID(int* data);
//motor config function


#define sampleFreq 100

volatile float xLoc;
volatile float yLoc;
volatile float angle; //goaling aiming angle
volatile float currAngle; //current angle in static frame
double diffAngle; //for PID, P term
volatile float lastCurrAngle; //last angle from the sensor, for PID, D term
volatile float w; //angular velocity

int xGoal;
int yGoal;
int firstRun = 1;
int attackingRight = 0;


volatile int sampleGet;

//Main Function
int main(void)
{
    m_clockdivide(0);
    sei();
    m_green(ON);
    m_wii_open();
    leftMotorIni();
    RightMotorIni();
    MotorIni();
    init_ADC();
    m_usb_init();
    m_bus_init();
    sampleTimer();
    m_green(OFF);
    m_disableJTAG();
    while(1)
    {
        STATUE = PLAY;
        
        if (STATUE == GOALB||STATUE == GOALR||STATUE == PAUSE||STATUE == HALFTIME||STATUE == GAMEOVER)
        {	OCR1B = 0;
            OCR1C = 0;
        }else{
            if (STATUE == COMMTEST)
            {
                m_red(ON);
                m_wait(500);
                m_red(OFF);
                m_wait(500);
                m_red(ON);
                m_wait(500);
                m_red(OFF);
                m_wait(500);
            }
            if (STATUE == PLAY)
            {
                
                    //F0 Corresponding Left, F1 Corresponding Right
                    ADCConversion(phoData);
                    m_wii_read(blobs);
                    getLocation();
                
                    gettingPuck(phoData);
                    
                    //m_usb_tx_string("f0:   ");
                    //m_usb_tx_int(phoData[0]);
                    //m_usb_tx_string("   f1:   ");
                    //m_usb_tx_int(phoData[1]);
                    //m_usb_tx_string("   f5:   ");
                    //m_usb_tx_int(phoData[3]);
                    //m_usb_tx_string("   f6:   ");
                    //m_usb_tx_int(phoData[4]);
                    //m_usb_tx_string("   f7:   ");
                    //m_usb_tx_int(phoData[5]);
                    //m_usb_tx_string("   d4:   ");
                    //m_usb_tx_int(phoData[6]);
                    //m_usb_tx_string("   f4:   ");
                    //m_usb_tx_int(phoData[2]);
                    //m_usb_tx_string("\n");
            }
        }
    }
}

void checkStatus(void){
    if (buffer[0] == 0xA0)
    {
        STATUE = COMMTEST;
    }
    if (buffer[0] == 0xA1)
    {
        STATUE = PLAY;
    }
    if (buffer[0] == 0xA2)
    {
        STATUE = GOALR;
    }
    if (buffer[0] == 0xA3)
    {
        STATUE = GOALB;
    }
    if (buffer[0] == 0xA4)
    {
        STATUE = PAUSE;
    }
    if (buffer[0] == 0xA6)
    {
        STATUE = HALFTIME;
    }
    if (buffer[0] == 0xA7)
    {
        STATUE = GAMEOVER;
    }
}

void gettingPuck(int* data){
    
    static int diff;
    static int old_diff = 0;
    static double kp = 0.8;
    static double kd = 20;
    int offsetOfOCR = 1500;
    static int old_kd;
    // For Testing On 18 & 19, We Use Data 4 And Data 6
    diff = data[4] - data[6];
    m_usb_tx_string("diff:   ");
    m_usb_tx_int(diff);
    
    
    int kd_num = kd * (-old_diff + diff);
    int kd_term = kd_num * 0.05 + 0.95*old_kd;
    old_kd = kd_num;
    double value =(double)kp*(- diff ) + kd_term;
    m_usb_tx_string("kd term:   ");
    m_usb_tx_int(kd_term);
    m_usb_tx_string("\n");
    
    
    int number_kp  = kp*(- diff );
    old_diff = diff;
    
    if (value >= 0) {
        set(PORTD,6);
        set(PORTD,7);
        OCR1B = offsetOfOCR + value;
        OCR1C = offsetOfOCR + value;
    }
    if (value < 0) {
        clear(PORTD,6);
        clear(PORTD,7);
        OCR1B = offsetOfOCR - value;
        OCR1C = offsetOfOCR - value;
    }
    
    
    
}

void getLocation(void){
    //read position
    
    //Pre-processing blobs matrix
    
    int i,j,k;
    int missNum = 0;
    int foundLoc = 0;
    
    stars[0][X] = blobs[0];    //get x-position
    stars[0][Y] = blobs[1];  //get y-position
    stars[1][X] = blobs[3];    //get x-position
    stars[1][Y] = blobs[4];  //.////get y-position
    stars[2][X] = blobs[6];    //get x-position
    stars[2][Y] = blobs[7];  //get y-position
    stars[3][X] = blobs[9];    //get x-position
    stars[3][Y] = blobs[10];  //get y-position
    
//    m_usb_tx_string("X1:   ");
//    m_usb_tx_int(blobs[0]);
//    m_usb_tx_string("   Y1:   ");
//    m_usb_tx_int(blobs[1]);
//    m_usb_tx_string("   X2:   ");
//    m_usb_tx_int(blobs[3]);
//    m_usb_tx_string("   Y2:   ");
//    m_usb_tx_int(blobs[4]);
//    m_usb_tx_string("   X3:   ");
//    m_usb_tx_int(blobs[6]);
//    m_usb_tx_string("   Y3:   ");
//    m_usb_tx_int(blobs[7]);
//    m_usb_tx_string("   X4:   ");
//    m_usb_tx_int(blobs[9]);
//    m_usb_tx_string("   Y4:   ");
//    m_usb_tx_int(blobs[10]);
//    m_usb_tx_string("\n");
    
    for(i=0; i<4; i++){
        if (stars[i][X] == 1023 && stars[i][Y] == 1023){missNum++;}
    }
    
    
    if(missNum < 2){
        for (i=0; i<4; i++)
        {
            if (stars[i][X] == 1023 && stars[i][Y] == 1023){continue;}                              //getting the first valid data
            if (foundLoc == 1){break;}
            
            //entering the loop for finding the longest mid distance
            for (j=0; j<4; j++)
            {
                if (stars[j][X] == 1023 && stars[j][Y] == 1023){continue;}                      //getting the second valid data
                if (j == i){continue;}
                if (foundLoc == 1){break;}
                
                float midEdgeCurr = getDist(stars[i],stars[j]);
                
                if (midEdgeCurr>=(midEdge-floatDis) && midEdgeCurr<=(midEdge+floatDis))                 //getting the mid edge points
                {
                    
                    
                    
                    for (k=0; k<4; k++)
                    {
                        if (stars[k][X] == 1023 && stars[k][Y] == 1023){continue;}
                        if (k == i){continue;}
                        if (k == j){continue;}
                        if (foundLoc == 1){break;}
                        
                        float EdgeCurr1 = getDist(stars[i],stars[k]);
                        float EdgeCurr2 = getDist(stars[j],stars[k]);
                        
                        if (EdgeCurr1 < EdgeCurr2)                                                              //justifying north and south point, i is north, j is south
                        {
                            
                            double ux = (double)(stars[i][X]-stars[j][X]);
                            double uy = (double)(stars[i][Y]-stars[j][Y]);
                            
                            currAngle = atan2(-ux, uy);
                            
                            m_usb_tx_string("   current angle:");
                            m_usb_tx_int(currAngle*100);
                            m_usb_tx_string("\n");
                            
                            
                            int xDiff = (float)0.689*(stars[i][X]-stars[j][X]);
                            int yDiff = (float)0.689*(stars[i][Y]-stars[j][Y]);
                            xLoc = (int)stars[j][X] + xDiff - 512;
                            yLoc = (int)stars[j][Y] + yDiff - 384;
                            
                            float xLocidx = xLoc;
                            
                            xLoc = 32*cos(currAngle) + 5*sin(currAngle) - 2*xLoc*cos(currAngle) - 2*yLoc*sin(currAngle); //location with respect to the pixel center
                            yLoc = 5*cos(currAngle) - 32*sin(currAngle) - 2*yLoc*cos(currAngle) + 2*xLocidx*sin(currAngle);
                            currAngle = -currAngle + 3.14;
                            xLoc = xLoc + 512; //location
                            yLoc = yLoc + 384;
                            foundLoc = 1;
                        }
                    }
                }
            }
            
            
            
            //Only three stars
            for (j=0; j<4; j++){
                if (stars[j][X] == 1023 && stars[j][Y] == 1023){continue;}                      //getting the second valid data
                if (j == i){continue;}
                if (foundLoc == 1){break;}
                
                unsigned int horiEdgeCurr = getDist(stars[i],stars[j]);
                if (horiEdgeCurr>=(horiEdge-floatDis) && horiEdgeCurr<=(horiEdge+floatDis))
                {
                    for (k=0; k<4; k++)
                    {
                        if (stars[k][X] == 1023 && stars[k][Y] == 1023){continue;}
                        if (k == i){continue;}
                        if (k == j){continue;}
                        if (foundLoc == 1){break;}
                        
                        unsigned int EdgeCurr3 = getDist(stars[i],stars[k]);
                        if (EdgeCurr3<55)                                                                                       //subjecting to change, 50 is the threshold for justifying
                        {                                                                                                                       //whether the third point is north or south point
                            //The third point is north point
                            unsigned int EdgeCurr4 = getDist(stars[j],stars[k]);
                            if (EdgeCurr3>EdgeCurr4)
                            {
                                //i is east point
                                double ux = (double)(stars[j][X]-stars[i][X]);											//Current unit vector from east to west
                                double uy = (double)(stars[j][Y]-stars[i][Y]);
                                
                                currAngle = atan2(-ux,uy);                                                              //the angle is between -pi to pi
                                
                                currAngle = currAngle - 1.23;
                                if (currAngle < -3.14) {
                                    currAngle = currAngle + 6.28;
                                }
                                
                                
                                int xDiff = 0.457*(stars[j][X]-stars[i][X]);
                                int yDiff = 0.457*(stars[j][Y]-stars[i][Y]);
                                
                                xLoc = stars[i][X] + xDiff - 512;
                                yLoc = stars[i][Y] + yDiff - 384;
                                
                                float xLocidx = xLoc;
                                
                                xLoc = 32*cos(currAngle) + 5*sin(currAngle) - 2*xLoc*cos(currAngle) - 2*yLoc*sin(currAngle); //location with respect to the pixel center
                                yLoc = 5*cos(currAngle) - 32*sin(currAngle) - 2*yLoc*cos(currAngle) + 2*xLocidx*sin(currAngle);
                                
                                currAngle = -currAngle + 3.14;
                                
                                xLoc = xLoc+512; //location
                                yLoc = yLoc+384;
                                foundLoc = 1;
                            }
                        }
                        if (EdgeCurr3>55)
                        {
                            //The third point is south point
                            unsigned int EdgeCurr4 = getDist(stars[j],stars[k]);
                            if (EdgeCurr3>EdgeCurr4)
                            {
                                //i is west point
                                double ux = (double)(stars[i][X]-stars[j][X]);											//Current unit vector from east to west
                                double uy = (double)(stars[i][Y]-stars[j][Y]);
                                
                                currAngle = atan2(-ux,uy);                                                              //the angle is between -pi to pi
                                
                                currAngle = currAngle - 1.23;
                                if (currAngle < -3.14) {
                                    currAngle = currAngle + 6.28;
                                }
                                
                                int xDiff = 0.457*(stars[i][X]-stars[j][X]);
                                int yDiff = 0.457*(stars[i][Y]-stars[j][Y]);
                                
                                xLoc = stars[j][X] + xDiff - 512;
                                yLoc = stars[j][Y] + yDiff - 384;
                                
                                float xLocidx = xLoc;
                                
                                xLoc = 32*cos(currAngle) + 5*sin(currAngle) - 2*xLoc*cos(currAngle) - 2*yLoc*sin(currAngle); //location with respect to the pixel center
                                yLoc = 5*cos(currAngle) - 32*sin(currAngle) - 2*yLoc*cos(currAngle) + 2*xLocidx*sin(currAngle);
                                
                                currAngle = -currAngle + 3.14;
                                
                                xLoc = xLoc+512; //location
                                yLoc = yLoc+384;
                                foundLoc = 1;
                                
                            }
                        }
                    }
                }
            }
            
            
            
        }
    }
    
    
    
    if (missNum >= 2)
    {
        currAngle = currAngle;
        xLoc = xLoc;
        yLoc = yLoc;
    }
    
}
void goaldirection(void){
    
    int xd = xGoal-xLoc;
    int yd = yGoal-yLoc;
    
    angle = atan2(yd,xd);
    
    
}

void leftMotorIni(void){
    set(TCCR1B,WGM13);
    set(TCCR1B,WGM12);
    set(TCCR1A,WGM11);
    set(TCCR1A,WGM10);
    
    clear(TCCR1B,CS12);
    clear(TCCR1B,CS11);
    set(TCCR1B,CS10);
    
    set(TCCR1A,COM1B1);
    clear(TCCR1A,COM1B0);
    
    set(DDRB,6);
}
void RightMotorIni(void){
    set(TCCR1A,COM1C1);
    clear(TCCR1A,COM1C0);
    
    set(DDRB,7);
}
void MotorIni(void){
    set(DDRD,6);                                                                            //initially config direction control
    set(DDRD,7);
    OCR1A = 2000;
    OCR1B = 0;
    OCR1C = 0;
}
void sampleTimer(void){
    set(TCCR3B,CS32); //256 prescalar
    set(TCCR3B,WGM32); //mode4 up to OCR3A
    OCR3A = 62500/sampleFreq;
    set(TIMSK3,OCIE3A);
}

//Math Function
int getDist(int a[], int b[]){
    int dx = abs(a[X]-b[X]);
    int dy = abs(a[Y]-b[Y]);
    return sqrt(dx*dx+dy*dy);
}

//Motor Config Function


//Interrupts
ISR(TIMER3_COMPA_vect){
    sampleGet = 1;
}
ISR(INT2_vect){
    m_rf_read(buffer,PACKET_LENGTH);       // pull the packet
    m_green(TOGGLE);
    checkStatus();
}
