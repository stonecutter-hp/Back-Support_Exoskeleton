/***********************************************************************
 * The Timer configuration and processing function 
 * for timer usage and PWM generation
 **********************************************************************/

#include "Timers.h"

extern uint16 PWMperiod_L = 500;    // period of left PWM  
extern uint16 PWMperiod_R = 500;    // period of right PWM 


/**
 * Initialize timer for controller, ADC update and Sending data to PC update
 *                     Timer3(CH4), Timer3(CH4),   Timer4(CH3)(PB8)      
 */
void Timers_Init(void) {
  Timer3.init();   // stop the timers before configuring them
  Timer4.init();   // stop the timers before configuring them
  // high-level control frequency
  HLUpdateFre = 72000/TIM4preScale/TIM4_OverflowValue*1000;
  // Configurate timer 3    
  Timer3.setPrescaleFactor(TIM3preScale);          // set pre scale factor 72 -->  72MHz/72=1MHz
  Timer3.setOverflow(TIM3_OverflowValue);          // set overflow value to determine timer 3 frequency: 1(MHz)/overflow
  Timer3.setCompare(TIM3_CH4,TIM3_OverflowValue);  // set compare value within overflow for frequency of timers interrupt
  Timer3.attachInterrupt(TIM3_CH4,Timer3_4_int);   // attach interrupt to timer3 channel4
  Timer3.refresh();                                // refresh the timer 3 configuration
  // Configurate timer 4    
  Timer4.setPrescaleFactor(TIM4preScale);          // set pre scale factor 72 -->  72MHz/72=1MHz
  Timer4.setOverflow(TIM4_OverflowValue);          // set overflow value to determine timer 4 frequency: 1(MHz)/overflow
  Timer4.setCompare(TIM4_CH3,TIM4_OverflowValue);  // set compare value within overflow for frequency of timers interrupt
  Timer4.attachInterrupt(TIM4_CH3,Timer4_3_int);   // attach interrupt to timer4 channel3
  Timer4.refresh();                                // refresh the timer 4 configuration
}

/**
 * Initialize timer for PWM generation
 */
void PWMmode_Init(void) {
  Timer1.pause();             // stop the timers before configuring them  (?)
  Timer2.init();              // stop the timers before configuring them
  pinMode(MotorPWM_L,PWM);    // assign PWM pin mode
  pinMode(MotorPWM_R,PWM);  
  // Configuration timer1 channel1  2khz
  Timer1.setPrescaleFactor(72);           // counting frequency = 72/72 = 1MHz 
  Timer1.setOverflow(PWMperiod_L);        // counting 500 with 1MHz = 0.5ms  2kHz
  Timer1.setCompare(TIM1_CH1,PWM_commandL);
  Timer1.refresh();  
  // Configuration timer2 channel2  2khz
  Timer2.setPrescaleFactor(72);           // counting frequency = 72/72 = 1MHz 
  Timer2.setOverflow(PWMperiod_R);        // counting 500 with 1MHz = 0.5ms  2kHz
  Timer2.setCompare(TIM2_CH2,PWM_commandR);
  Timer2.refresh(); 
}

/************************************* Interrupt handling function of timer3 and timer4 *********************************************/
/**
 * Interruption of timer3 is for ADC update and Control update
 * Frequency = 72/TIM3preScale/TIM3_OverflowValue = 72MHz/72/1000 = 1kHz
 */
void Timer3_4_int(void) {
  if(ADC_update == false) {
    ADC_update = true;
  }
  if(Control_update == false) {
    Control_update = true;
  }
}

/**
 * Interruption timer4 is for data sending to PC
 * Frequency = 72/TIM4preScale/TIM4_OverflowValue = 72MHz/72/2500 = 400Hz
 */
void Timer4_3_int(void) {
  if(SendPC_update == false) {
    SendPC_update = true;
  }
  if(HLControl_update == false) {
    HLControl_update = true;
  }
}
