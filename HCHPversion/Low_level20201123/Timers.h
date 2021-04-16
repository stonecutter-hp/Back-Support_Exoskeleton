/***********************************************************************
 * The Timer configuration and processing function 
 * for timer usage and PWM generation
 **********************************************************************/

#ifndef __TImer_H__
#define __TImer_H__

#include "Arduino.h"
#include "Control.h"

/*********************************** Timer configuration parameter definition ************************************/
// Timer3(CH4) is assigned for ADC and Control(PWM) command frequency arrangement
// Timer4(CH3)(PB8) is assigned for frequency of data sending to PC
// Notice the Overflow value should range in 0~65535
#define TIM3_CH4 4               // timer3 channel4
#define TIM3preScale 72          // 72MHz/72 = 1Mhz
#define TIM3_OverflowValue 1000  // 1Mhz/1000 = 1kHz
#define TIM4_CH3 3               // timer4 channel3
#define TIM4preScale 72          // 72MHz/72 = 1MHz
#define TIM4_OverflowValue 2500  // 1Mhz/5000 = 200Hz

/**************************************** PWM related timers parameter definition ********************************/
// Timer1_CH1(PA8) is assigned for the first channel PWM for left motor
// Timer2_CH2(PA1) is assigned for the seconed channel PWM for right motor
#define MotorPWM_L PA8    // pin for left PWM
#define MotorPWM_R PA1    // pin for right PWM
#define TIM1_CH1 1  // timer1 channel1
#define TIM2_CH2 2  // timer2 channel2
// reminder the value should not exceed 65535
// the PWM pulse value is define in control part
extern uint16 PWMperiod_L;    // period of left PWM  
extern uint16 PWMperiod_R;    // period of right PWM 


/**
 * Initialize timer for controller, ADC update and Sending data to PC update
 */
void Timers_Init(void);

/**
 * Initialize timer for PWM generation
 */
void PWMmode_Init(void);

/**
 * Interruption of timer3 is for ADC update and Control update
 * Frequency = 72/TIM3preScale/TIM3_OverflowValue = 72MHz/72/1000 = 1kHz
 */
void Timer3_4_int(void);

/**
 * Interruption timer4 is for data sending to PC
 * Frequency = 72/TIM4preScale/TIM4_OverflowValue = 72MHz/72/2500 = 400Hz
 */
void Timer4_3_int(void);

#endif
