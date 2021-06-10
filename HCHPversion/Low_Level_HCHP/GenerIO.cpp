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
  pinMode(MotorEnableR, OUTPUT);
  pinMode(MotorRotationR, OUTPUT);
  // Initial pin status assignment fot left motor
  digitalWrite(MotorRotationL,HIGH);  // ensure the correct rotation direction (Cable Pulling Direction)
  digitalWrite(MotorEnableL,LOW);    // initially disable motor 
  // Initial pin status assignment for right motor
  digitalWrite(MotorRotationL,LOW);  // ensure the correct rotation direction (Cable Pulling Direction)
  digitalWrite(MotorEnableL,LOW);    // initially disable motor 
}
