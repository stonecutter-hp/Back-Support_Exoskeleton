/***********************************************************************
 * General IO assignment and initialization
 **********************************************************************/

#ifndef __GnerIO___H
#define __GnerIO___H

#include "Arduino.h"

#define MotorEnableL PB9       //Left Motor Enable pin
#define MotorRotationL PB10    //Left Motor Rotation Direction pin
#define MotorCurrentFlagL PB11 //Left Motor Detection Flag
#define MotorEnableR PB3       //Right Motor Enable pin
#define MotorRotationR PB4     //Right Motor Rotation Direction pin
#define MotorCurrentFlagR PB5  //Right Motor Detection Flag
#define LED0 PB0               //PB0-green light
#define LED1 PB1               //PB1-red light

/**
 * Assign general IO for motor control and LED state indication
 */
void GeneralIO_Init(void);







#endif 
