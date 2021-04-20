#include <AD7173.h>  // the library for ADC
#include "ADC.h"
#include "Control.h"
#include "IIC.h"
#include "IMU.h"
#include "SerialComu.h"
#include "Timers.h"

/*  Program try 1 for the low-level control of ZXHP version exoskelton 
    prototype which is aiming to run at test bench with only one side
    of torque transmission system
Log
20210416
  Creating based on "Low-level20201123.ino"
  For the first version, the high-level control part of torque generation 
  and user intention detection is embeded in the low-level controller for 
  simple implementation
  As a result, only sendDatatoPC is needed and no command receiving from PC
20210420
  The user intention detection strategy refers to the rule-based strategy written
  in the 'Thoughs Keeping notebook'
  

***Program logic***
1) Read sensor feedback including: 
    potentiometer for hip angle feedback, 
    motor driver for motor current and velocity feedback (for potential cascaded control), 
    force sensor for interaction force indication,
    IMU for human trunk angle feedback -->
2) Simple user intention detection and reference torque generation calculation -->
3) calculate the actual control commmand for motor -->
4) send sensor feedback to PC.
*/

void setup() {
  /******************************** Serial Initialization ******************************/
  Serial.begin(460800);   // initialize serial set baurd rate

  /*************************** Control parameter Initialization ************************/
  Control_Init();  // initialize the control parameters
  
  /*********** Assign general IO for motor control and LED state indication ************/
  GeneralIO_Init();

  /****************** ADC initialization for channel mode configuration ****************/
  ADC_Init();
  Filter_Init();  // Initial ADC feedback storing/processing vector/matrix

  /******************************** IIC Initialization *********************************/
  IIC_Init();     // IIC initialization for IMU communication  

  /******************************** IMU Initialization *********************************/
  IMU_Init();     // Initial IMU feedback storing vector

  /******************************** Timer Initialization *******************************/
  Timers_Init();

  /***************************** PWM Generation Initialization *************************/
  PWMmode_Init();

  // resume all the timers
  Timer1.resume();     // Motor L PWM
  Timer2.resume();     // Motor R PWM
  Timer3.resume();     // ADC and control update
  Timer4.resume();     // Sending PC update

  /******************* ADC value and IMU angle feedback initialization *****************/
  getADCaverage(1);
  getIMUangleL();
  yawAngleR20(1);     // Forced trunk yaw angle correction
  HipAngL_InitValue = Aver_ADC_value[PotentioLP1];  // Hip angle initial vallue
  delay(5); 
}


void loop() {
//  unsigned long starttime;
//  unsigned long stoptime;
//  unsigned long looptime;
//  starttime = millis();
  if(ADC_update) {
  	getADCaverage(1);              // get ADC value
  	ADC_update = false;
  }
  getIMUangleT();                  // get human trunk flexion angle
  getIMUvelT();                    // get human trunk flexion velocity
  // yawAngleR20(0);               // trunk yaw angle correction, should before data sensor feedback processing and sending
  sensorFeedbackPro();             // processing sensor feedback for high-level control and low-level closed-loop control 
  // MovingAverFilterIMUC(3,rollChan);   // Averaged moving filtered
  if(HLControl_update) {
    HLControl(1,1);                // At present the frequency of high-level control is the same as data sending frequency 
    HLControl_update = false;
  }
  if(Control_update) {
  	Control(1);                    // calculate controlled command: PWM duty cycles
  	Control_update = false;        // At present the frequency of low-level control is the same as ADC sensor feedback update frequency
  }
  if(SendPC_update) {
  	sendDatatoPC();                // send sensor data to PC and allow next receiving cycle
    SendPC_update = false;
  } 
//  stoptime = millis();
//  looptime = stoptime - starttime;
//  Serial.println(looptime);
//  while(1);
}