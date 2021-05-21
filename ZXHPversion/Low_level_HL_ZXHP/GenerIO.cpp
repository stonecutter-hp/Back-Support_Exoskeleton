/***********************************************************************
 * General IO assignment and initialization
 **********************************************************************/
 
 #include "GenerIO.h"


/**
 * Assign general IO for motor control and LED state indication
 */
void GeneralIO_Init(void) {
  // Pin assignment
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(MotorEnableL, OUTPUT);
  pinMode(MotorRotationL, OUTPUT);
  // Initial pin status assignment
  digitalWrite(MotorRotationL,LOW);  // ensure the correct right rotation direction (CCW for work bench)
  digitalWrite(MotorEnableL,LOW);    // initially not enable motor  
}
