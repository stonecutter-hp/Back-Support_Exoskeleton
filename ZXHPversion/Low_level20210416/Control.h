/***********************************************************************
 * The PID control configuration and processing function 
 * System model parameters
 **********************************************************************/

#ifndef __COntrol__H
#define __COntrol__H

#include "Arduino.h"
#include "SerialComu.h"
#include "Timers.h"
#include "ADC.h"
#include "IMU.h"

/**************************************** Low level PID control parameters definition ********************************/
// Here use increment PID algorithm: Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
typedef struct
{
  float currT;  //current torque feedback
  float set;    //deseried torque

  float Err;    //error
  float Err_p;  // last time error
  float Err_pp; // last last time error
  
  float Kp;       // Propotional coefficient
  float Ti;       // Time constant of integration part
  float Td;       // Time constant of derivative part
  float Tcontrol; // Control period, every Tcontrol MCU update once output command
  
  float Delta_Ta;       // PID control output of actuation torque
  float Delta_Current;  // PID control output of corresponding motor current 
  float Delta_PWM;      // Calculation results
  float currpwm;        // Current PWM duty width
  float currTa;         // Current actuation torque
  float currCurrent;    // Current motor driving curren
  uint16_t pwm_cycle;   // The whole period of PWM
}PID;    // PID parameter structure

extern PID pidL;  // control parameter of left motor
extern PID pidR;  // control parameter of right motor
// the desired torque from PC is defined in communication receiving parameter part
extern int16_t PWM_commandL;   // range: 0.1*PWMperiod_L~0.9*PWMperiod_L
extern int16_t PWM_commandR;   // range: 0.1*PWMperiod_R~0.9*PWMperiod_R
extern bool Control_update;    // control update flag
 
// Previously workable Kp/Ki/Kd for test bench with large torsion spring:
// 0.58/0/0.28  0.68/0/0.3
#define KP_L 0.5              // Kp for PID control of motor L
#define KI_L 0.2              // Ki for PID control of motor L
#define KD_L 0.28             // Kd for PID control of motor L
#define KP_R 1                // Kp for PID control of motor R
#define KI_R 0.002            // Ki for PID control of motor R
#define KD_R 0.00267          // Kd for PID control of motor R
#define LimitDelta_TaL 500000 // Limitation of delta control command of motor L
#define LimitTotal_TaL 7      // Limitation of total control command of motor L
#define LimitDelta_TaR 500000 // Limitation of delta control command of motor R
#define LimitTotal_TaR 7      // Limitation of total control command of motor R
#define LimitInput 15         // Limitation of input command, here for open-loop is Ta, for closed loop is Td

/****************************************High level controller parameters definition*******************************/
extern bool HLControl_update;  // high-level control update flag
// Controller parameter and threshold for high-level control strategy, the content may be adjusted according to future
// practical applied high-level strategy
typedef struct {
  float ThrTrunkFleAng;    // Threshold for trunk flexion angle
  float ThrTrunkFleVel;    // Threshold for trunk flexion velocity
  float ThrHipAngMean;     // Threshold for mean value of summation of left and right hip angle  
  float ThrHipAngDiff;     // Threshold for difference between left and right hip angle
  float ThrHipAngStd;      // Threshold for standard deviation of mean hip angle
  float ThrHipVel;         // Threshold for mean hip angle velocity
  float ThrThighAngMean;   // Threshold for mean value of summation of left and right thigh angle
  float ThrThighAngStd;    // Threshold for standard deviation of mean thigh angle
  float ThrThighAngVel;    // Threshold for mean thigh angle velocity
  float ThrHipAngDiffStd;  // Threshold for standard deviation of difference between left and right hip angle
    
}HLCont;  // Controller parameter and threshold for high-level control strategy
extern HLCont HL_HP;       // Parameters for specified subjects

/**************************************** Transmissio system parameters definition ********************************/
// The output ability of the actuation unit with 19:1 gear ratio is better restricted to 0~8.9 Nm (0.0525*9*19)
// The following parameter may be adjusted after calibrateds
#define MotorCurrentConstant 0.0437      //motor current constant Nm/A
#define MotorMaximumCurrent 9            //motor maximum current A configured in EXCON studio
#define GearRatio 19                     //gear ratio is 19:1
#define ForceSensorL_Sensitivity 1       //for force sensor calibration
#define LoadCellL_Sensitivity 0.00166    //for load cell calibration
#define PotentioLP1_Sensitivity 0.0083   //0.0083 = 2.5/300 (v/deg); for potentiometer calibration 

/*************************************** Intermediate auxiliary parameters for control ****************************/ 
// Parameters for lowe-level control
extern float Estimated_TdMotorCurrentL;   // Td feedback from left motor current feedback
extern float Estimated_TdMotorCurrentR;   // Td feedback from right motor current feedback
extern float Estimated_TdForceSensorL;    // Td feedback from left force sensor
extern float Estimated_TdForceSensorR;    // Td feedback from right force sensor
extern float Estimated_TdL;               // Estimated compact Td feedback of left side
extern float Estimated_TdR;               // Estimated compact Td feedback of right side
// Parameters for high-level controller (Directly feedback from sensor)
extern float HipAngL;                     // Left hip joint angle
extern float HipAngL_InitValue;           // Auxiliary parameter for left hip joint angle
extern float HipAngR;                     // Right hip joint angle
extern float HipAngR_InitValue;           // Auxiliary parameter for right hip joint angle
extern float TrunkYawAng;                 // Trunk yaw angle
extern float TrunkYaw_InitValue;          // Auxiliary parameter for trunk yaw angle
extern float TrunkFleAng;                 // Trunk flexion angle
extern float TrunkFleAng_InitValue;       // Auxiliary parameter for trunk pitch angle
extern float TrunkFleVel;                 // Trunk flexion angular velocity
// Parameters for high-level controller (Calculated from sensor feedback)
extern float HipAngMean;                  // (Left hip angle + right hip angle)/2
extern float HipAngDiff;                  // (Left hip angle - right hip angle)
extern float HipAngStd;                   // Std(HipAngMean) within certain time range
extern float HipAngVel;                   // Velocity of HipAngMean
extern float ThighAngL;                   // Left thigh angle
extern float ThighAngR;                   // Right thigh angle
extern float ThighAngMean;                // (Left thigh angle + right thigh angle)/2
extern float ThighAngStd;                 // Std(ThighAngMean) within certain time range
extern float HipAngDiffStd;               // Std(HipAngDiff) within certain time range


/**
 * Control parameter initialization for Low-level controller
 * Initial controller including command and controller parameters for Low-level PID controller/ Open-loop controller
 * Here use increment PID algorithm: Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
 */
void Control_Init(void);

/**
 * Control parameter initialization for High-level controller
 * Initial controller including sensor feedback, auxiliary parameters and thresholds used for high-level controller
 */
void HLControl_Init(void);

/**
 * Set the yaw angle of human trunk to zero
 * @param unsigned char - control mode: 1-9 axis IMU 2-6 axis IMU
 */
void yawAngleR20(uint8_t aloMode);

/**
 * Processing sensor feedback for closed-loop control and data sending to PC
 */
void sensorFeedbackPro(void);

/**
 * Conduct simple user intention detection and torque generation calculation as reference torque 
 * for low-level control based on sensor information feedback from force sensors, IMUs, Potentiometers
 * and Motor driver
 * @para unsigned char - control mode 1 for user intenntion detection: 1-xx algorithm, 2-xx algorithm
 *       unsigned char - control mode 2 for torque generation strategy: 1-xx strategy, 2-xx strategy
 */
void HLControl(uint8_t UIDMode, uint8_t RTGMode);

/**
 * Conduct simple user intention detection 
 * @para unsigned char - control mode for user intenntion detection: 1-xx algorithm, 2-xx algorithm
 */
void HL_UserIntentDetect(uint8_t UIDMode);

/**
 * Reference torque generation 
 * @para unsigned char - control mode for torque generation strategy: 1-xx strategy, 2-xx strategy
 */
void HL_ReferTorqueGenerate(uint8_t RTGMode);

/**
 * Calculate control command (PWM duty cycle) accroding to command received from PC
 * and information received from ADC and IMU
 * @para unsigned char - control mode: 1-PID control, 2-Open loop control
 * Here use increment PID algorithm: Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
 */
void Control(uint8_t ContMode);

/**
 * Set the pwm duty cycle for both of the motors
 * @param unsigned int - PWMcommandL/PWMcommandR: compared value
 *        range in 0~PWMperiod_L/PWMperiod_R
 */
void MotorPWMoutput(uint16_t PWMcommandL, uint16_t PWMcommandR);








#endif
