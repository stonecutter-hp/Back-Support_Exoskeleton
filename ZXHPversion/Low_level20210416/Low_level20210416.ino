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
  // yawAngleR20(1);     // Forced trunk yaw angle correction,
  PotentioLP1_InitValue = Aver_ADC_value[PotentioLP1];
  delay(5); 
}


void loop() {
//  unsigned long starttime;
//  unsigned long stoptime;
//  unsigned long looptime;
//  starttime = millis();
  receiveDatafromPC();         // receive data from PC
  receivedDataPro();           // decomposite data received from PC
  if(ADC_update) {
  	getADCaverage(1);          // get ADC value
  	ADC_update = false;
  }
  // getIMUangle();            // get rotation angle of both support beam and human back/link
  getIMUangleT();              // get support beam rotation angle from IMU
  // yawAngleR20(0);              // trunk yaw angle correction, should before data sensor feedback processing and sending
  sensorFeedbackPro();         // processing sensor feedback for closed-loop control 
  // MovingAverageFilter(2);   // Averaged moving filtered
  if(Control_update) {
  	Control(1);                // calculate controlled command: PWM duty cycles
  	Control_update = false;
  }
  if(SendPC_update) {
  	sendDatatoPC();            // send sensor data to PC and allow next receiving cycle
    SendPC_update = false;
    receiveCompleted = false;  // Mark this correct receiving infomation is used up 
  }
    receiveContinuing = true;  // Enable next time's recieving
    USART_RX_STA = 0;          // Return to zero for receiving buffer 
//  stoptime = millis();
//  looptime = stoptime - starttime;
//  Serial.println(looptime);
//  while(1);
}
