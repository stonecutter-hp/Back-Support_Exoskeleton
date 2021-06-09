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
const double attRatio[16] = {1,  1,  1,  1,  22.5,   2,  2,  2,  2,  2,   1,   1,   1,   1,   1,   1};
#define ADC_SS PA4           //SPI1_NSS-CS 
#define ADC_MISO PA6         //SPI1_MISO-DOUT/RDY
#define ENABLED_CH 16        //sum of ADC Channels
#define MotorCurrL 6         //ADC channel assigned for left motor current feedback
#define MotorCurrR 8         //ADC channel assigned for right motor current feedback
#define MotorVeloL 5         //ADC channel assigned for left motor velocity feedback
#define MotorVeloR 7         //ADC channel assigned for right motor velocity feedback
#define PotentioLP1 10       //ADC channel assigned for left potentiometer 1(P1 left torsion spring rotation)
#define PotentioLP2 11       //ADC channel assigned for left potentiometer 2(P2 left hip angle)
#define PotentioRP3 12       //ADC channel assigned for right potentiometer 1(P3 right torsion spring rotation)
#define PotentioRP4 13       //ADC channel assigned for right potentiometer 2(P4 right hip angle)
#define LoadCellL 0          //ADC channel assigned for left load cell
#define LoadCellR 1          //ADC channel assigned for right load cell
#define FilterCycles 10      //FilterCycles for moving/exponetial average filter

// Parameters for ADC raw data processing for sensors
// Referring to the results from sensorCalibration.ino
#define LoadCellL_Sensitivity 0.00193    //0.00166    //0.00191(1.5mv/v)    //0.00227(1.78mv/v)  //from load cell calibration
#define LoadCellR_Sensitivity 0.00196    //0.00162    //0.00166    //from load cell calibration
#define LoadCellL_Offset 1.5               //offset of left load cell
#define LoadCellR_Offset 3.5               //offset of right load cell
#define PotentioLP1_Sensitivity 0.0080   //0.0077 = 2.5/325 (v/deg); from potentiometer calibration 
#define PotentioLP2_Sensitivity 0.0077   //0.0077 = 2.5/325 (v/deg); from potentiometer calibration 
#define PotentioLP3_Sensitivity 0.0077   //0.0077 = 2.5/325 (v/deg); from potentiometer calibration 
#define PotentioLP4_Sensitivity 0.0080   //0.0077 = 2.5/325 (v/deg); from potentiometer calibration 


/* ADC conversion data and STATUS register */
extern byte ADC_data[ENABLED_CH][4];             // store raw data from ADC
extern bool ADC_update;                          // ADC_update enable flag 
extern unsigned long ADC_value[ENABLED_CH];      // store ADC value in long format  
extern double Aver_ADC_value[ENABLED_CH];        // store the transferred ADC value for calculation
extern double Aver_ADC_value_unfiltered[ENABLED_CH][FilterCycles];
extern double Aver_ADC_value_Prev[ENABLED_CH];


/**
 * ADC initialization for channel mode configuration
 */
void ADC_Init(void); 

/**
 * Initial the matrix for average filter Aver_ADC_value_unfiltered[ENABLED_CH][FilterCycles]
 */
void Filter_Init(void);

/**
 * Mean Moving filter for the ADC value
 * @param int - channel: 0~ENABLED_CH-1
 * @param int - cycles: 1~FilterCycles
 */
void MovingAverageFilter(int channel, int cycles);

/**
 * Exponential moving average filter for the ADC value
 * @param int - channel: 0~ENABLED_CH-1
 * @param float - alpha: weighting decreasing coefficient (0~1)
 */
void ExponentialMovingFilter(int channel, float alpha);

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
