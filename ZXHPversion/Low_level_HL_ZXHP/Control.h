/***********************************************************************
 * The PID control configuration and processing function &
 * System model parameters
 **********************************************************************/

#ifndef __COntrol__H
#define __COntrol__H

#include "Arduino.h"
#include "SerialComu.h"
#include "Timers.h"
#include "ADC.h"
#include "IMU.h"
#include "FSM.h"
#include "RefTG.h"

#define d2r M_PI/180

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
extern int8_t PWMSignL;        // to mark the rotation direction of the left motor
extern int8_t PWMSignR;        // to mark the rotation direction of the right motor
extern bool Control_update;    // control update flag
 
// Previously workable Kp/Ki/Kd for test bench with large torsion spring:
// 0.58/0/0.28  0.68/0/0.3
#define KP_L 0.5              // Kp for PID control of motor L
#define KI_L 0.0000002        // Ki for PID control of motor L
#define KD_L 0.28             // Kd for PID control of motor L
#define KP_R 1                // Kp for PID control of motor R
#define KI_R 0.0000002        // Ki for PID control of motor R
#define KD_R 0.00267          // Kd for PID control of motor R
#define LimitDelta_TaL 500000 // Limitation of delta control command of motor L
#define LimitTotal_TaL 7      // Limitation of total control command of motor L
#define LimitDelta_TaR 500000 // Limitation of delta control command of motor R
#define LimitTotal_TaR 7      // Limitation of total control command of motor R
#define PWMUpperBound 0.75    // Upper bound of the PWM cycle duty
#define PWMLowerBound 0.12    // Lower bound of the PWM cycle duty
#define LimitInput 15         // Limitation of input command, here for open-loop is Ta, for closed loop is Td

/**************************************** Transmission system parameters definition ********************************/
// The output ability of the actuation unit with 19:1 gear ratio is better restricted to 0~8.9 Nm (0.0525*9*19)
// The following parameter may be adjusted after calibrateds
#define MotorCurrentConstant 0.0302      //motor current constant Nm/A
#define MotorMaximumCurrent 6            //motor maximum current A configured in EXCON studio
#define GearRatio 378                    //gear ratio is 126*3:1

/******************** Low-level controller related sensor feedback calibration value definition ********************/
// Expected initial value range (CaliValue +- Tol) of sensor feedback for initial calibration
// the initial values should be adjusted along with prototype design
#define TorqueSensorL_CaliValue 0
#define TorqueSensorL_Tol 0
#define TorqueSensorR_CaliValue 0
#define TorqueSensorR_Tol 0
#define ForceSensorL_CaliValue 0
#define ForceSensorL_Tol 0
#define ForceSensorR_CaliValue 0
#define ForceSensorR_Tol 0


/*************************************** Intermediate auxiliary parameters for control ****************************/ 
// Parameters for low-level control
extern float Estimated_TdMotorCurrentL;   // Td feedback from left motor current feedback
extern float Estimated_TdMotorCurrentR;   // Td feedback from right motor current feedback
extern float Estimated_TdTorqueL;         // Td feedback from left torque sensor
extern float TorqueL_InitValue;           // Auxiliary parameter for left torque sensor
extern float Estimated_TdTorqueR;         // Td feedback from right torque sensor
extern float TorqueR_InitValue;           // Auxiliary parameter for right torque sensor
extern float Estimated_TdForceSensorL;    // Td feedback from left force sensor
extern float ForceSensorL_InitValue;      // Auxiliary parameter for left force sensor
extern float Estimated_TdForceSensorR;    // Td feedback from right force sensor
extern float ForceSensorR_InitValue;      // Auxiliary parameter for right force sensor
extern float Estimated_TdL;               // Estimated compact Td feedback of left side
extern float Estimated_TdR;               // Estimated compact Td feedback of right side

/**
 * Control parameter initialization for Low-level controller
 * Initial parameters including: 
 * PWM commands; Iterative force feedback; PID struct parameters (PID controller parameters) related to PWM command.
 * Here use increment PID algorithm: Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
 */
void Control_Init(void);

/**
 * Control auxiliary parameter initialization
 * Initial parameters including: 
 * Interative force feedback
 */
void ControlAux_Init(void);

/**
 * Pre-processing for sensor feedback related to low-level controller 
 * to make sure the initial status of sensor is good for calibration
 * @return int8_t - Sensor ready flag: 0-Not Ready; 1-Ready
 */
int8_t LLPreproSensorInit(void);

/**
 * Pre-processing for sensor feedback related to both low-level controller 
 * and high-level control to make sure the initial status of sensor is good for calibration
 * @return int8_t - Sensor ready flag: 0-Not Ready; 1-Ready
 */
int8_t PreproSensorInit(void);

/**
 * Processing sensor feedback for Low-level closed-loop control
 */
void sensorFeedbackPro(void);

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
