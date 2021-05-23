#include <AD7173.h>  // the library for ADC
#include "ADC.h"
#include "Control.h"
#include "IIC.h"
#include "IMU.h"
#include "SerialComu.h"
#include "Timers.h"

/*  Program try 1 for the low-level control of HCHP version exoskelton 
    prototype which is aiming to run at test bench with only one side
    of torque transmission system
Log
20201123
  Creating based on the logical of "test_serial_ADC_timer.ino"
  Setup feedback item and parameters based on test-bench sensor implementation
20210423
  Keep updated with the programming of ZXHP version control program
20210520
  Upgrate it for application of HCHP version exoskeleton prototype control

***Program logic***
1) read desired torque command from PC 
    (if no command recieved, keep sending sensor feedback with fixed frequncy
    and use the initial/former command as reference command)--> 
2) read sensor feedback including: 
    potentiometers for torque feedback, 
    potentiometers for hip joint
    motor drivers for motor current and velocity feedback (for potential cascaded control), 
    load cells for cable force feedback,
    and a IMU for human back bending motion -->
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
  
  /***************************** Sensor initial value calibration **********************/
  LLPreproSensorInit();
  // resume all the timers
  Timer1.resume();     // Motor L PWM
  Timer2.resume();     // Motor R PWM
  Timer3.resume();     // ADC and control update
  Timer4.resume();     // Sending PC update
  delay(5); 
}


void loop() {
//  unsigned long starttime;
//  unsigned long stoptime;
//  unsigned long looptime;
//  starttime = millis();
  receiveDatafromPC();         // receive data from PC
  receivedDataPro();           // decomposite data received from PC
  /* For handshake with high-level controller with mode = Stop state */
  if(mode == 0) {
    // Ensure the initial state until successful handshake
    LLPreproSensorInit();
  }
  if(ADC_update) {
    getADCaverage(1);          // get ADC value
    getIMUangleT();            // get human trunk flexion angle
    getIMUvelT();              // get human trunk flexion velocity
    // MovingAverFilterIMUC(rollChan,3);   // Averaged moving filtered
    ADC_update = false;
  }
  if(Control_update) {
    // yawAngleR20(LogicInit, OperaitonAloIMUC);  // Here inside operation is for IMUC address
    sensorFeedbackPro();       // processing sensor feedback for closed-loop control 
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
