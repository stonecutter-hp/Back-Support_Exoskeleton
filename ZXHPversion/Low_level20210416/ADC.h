/***********************************************************************
 * The ADC pin definition and processing function
 **********************************************************************/

#ifndef _ADC__H__
#define _ADC__H__

#include <AD7173.h>
#include "GenerIO.h"

// Ch0~Ch3 is for load cell
// Ch4 is for battery voltage detection with attenuation ratio 1:11
// Ch5~Ch9 is for motor driver signal detection with attenuation ratio 1:2
// Ch10~Ch15 is for potentiometer with no attenuation
// --------------------- CH0 CH1 CH2 CH3   CH4   CH5 CH6 CH7 CH8 CH9 CH10 CH11 CH12 CH13 CH14 CH15
const int attRatio[16] = {1,  1,  1,  1,  22.5,   2,  2,  2,  2,  2,   1,   1,   1,   1,   1,   1};
#define ADC_SS PA4           //SPI1_NSS-CS 
#define ADC_MISO PA6         //SPI1_MISO-DOUT/RDY
#define ENABLED_CH 16        //sum of ADC Channels
#define MotorCurrL 6         //ADC channel assigned for left motor current feedback
#define MotorCurrR 8         //ADC channel assigned for right motor current feedback
#define MotorVeloL 5         //ADC channel assigned for left motor velocity feedback
#define MotorVeloR 7         //ADC channel assigned for right motor velocity feedback
#define PotentioLP1 10       //ADC channel assigned for left potentiometer 1(P1)
#define PotentioRP2 11       //ADC channel assigned for right potentiometer 1(P2)
#define ForceSensorL 12      //ADC channel assigned for left force sensor
#define ForceSensorR 13      //ADC channel assigned for right force sensor
#define LoadCellL 0          //ADC channel assigned for left load cell
#define LoadCellR 1          //ADC channel assigned for right load cell
#define FilterCycles 10      //FilterCycles for moving/exponetial average filter

/* ADC conversion data and STATUS register */
extern byte ADC_data[ENABLED_CH][4];             // store raw data from ADC
extern bool ADC_update;                          // ADC_update enable flag 
extern unsigned long ADC_value[ENABLED_CH];      // store ADC value in long format  
extern double Aver_ADC_value[ENABLED_CH];        // store the transferred ADC value for calculation
// Store inter value for moving average filter
extern double Aver_ADC_value_filtered[ENABLED_CH][FilterCycles];
// Store inter value for moving average & exponential filter
extern double Aver_ADC_value_Prev[ENABLED_CH];
/* load cell force transfer */
extern double LoadCell[4];                      // store the transferred force value


/**
 * ADC initialization for channel mode configuration
 */
void ADC_Init(void); 

/**
 * Initial the matrix for average filter Aver_ADC_value_filtered[ENABLED_CH][FilterCycles]
 */
void Filter_Init(void);

/**
 * Mean Moving filter for the ADC value
 * @param double - cycles: 1~FilterCycles
 */
void MovingAverageFilter(int cycles);

/**
 * Exponential moving average filter for the ADC value
 * @param double - weighting decreasing coefficient alpha (0~1)
 */
void ExponentialMovingFilter(double alpha);

/**
 * Get the ADC value of all channels once
 */
void getADC(void);

/**
 * Get average value from ADC for cycles
 * @para int - times: the cycles ADC go through before give you the detected value
 */
void getADCaverage(int times);



#endif
