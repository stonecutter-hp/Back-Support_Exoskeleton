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

#define d2r M_PI/180

/* Low level PID control parameters definition */
// Here use increment PID algorithm: 
// Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
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

/* Transmissio system parameters definition */
// The output ability of the actuation unit with 19:1 gear ratio is better restricted to 0~8.9 Nm (0.0525*9*19)
// The following parameter may be adjusted after calibrateds
#define MotorCurrentConstant 0.0437      // Nm/A, Motor current constant 
#define MotorMaximumCurrent 9            // A, Motor maximum current A configured in EXCON studio
#define GearRatio 19                     // Gear ratio
#define HumanBackLength 0.6              // m, Human back length

#define PulleyRadiusL 0.045              // m, Left side pulley radius
#define SupportBeamLengthL 0.5           // m, Length of left side support beam
#define TorsionStiffnessL 0.47           // Nm/deg, Left side torsion spring's stiffness
#define MechConsAngleL 80                // deg, Mechanical constraint setting angle of left side
#define PulleyRadiusR 0.045              // m, Right side pulley radius
#define TorsionStiffnessR 0.47           // Nm/deg, Right side torsion spring's stiffness
#define MechConsAngleR 80                // deg, Mechanical constraint setting angle of right side
#define SupportBeamLengthR 0.5           // m, Length of left side support beam

/* Sensor feedback calibration value definition */
// Expected initial value range (CaliValue +- Tol) of sensor feedback for initial calibration
// the initial values should be adjusted along with prototype design
#define TdL_CaliValue 0
#define TdL_Tol 0
#define TdR_CaliValue 0
#define TdR_Tol 0
#define HipAngL_CaliValue 0
#define HipAngL_Tol 0
#define HipAngR_CaliValue 0
#define HipAngR_Tol 0
#define FcL_CaliValue 0
#define FcL_Tol 0
#define FcR_CaliValue 0
#define FcR_Tol 0
#define Theta0_L 10             // deg, Iniital angle between left suppport beam and human back
#define Theta0_R 10             // deg, Iniital angle between right suppport beam and human back

/* Intermediate auxiliary parameters reagrding torque and hip angle feedback for low-level control */
extern float HipAngL;                    // deg, Left hip joint angle
extern float HipAngL_InitValue;          // deg, Auxiliary parameter for left hip joint angle
extern float Estimated_TdL;              // Nm, Td feedback of left side from torsion spring
extern float TdL_InitValue;              // Nm, Auxiliary parameter for left Td
extern float Estimated_FcL;              // N,  Cable force feedback of left side from load cell
extern float FcL_InitValue;              // N, Auxiliary parameter for left cable forve

extern float HipAngR;                    // deg, Right hip joint angle
extern float HipAngR_InitValue;          // deg, Auxiliary parameter for right hip joint angle
extern float Estimated_TdR;              // Nm, Td feedback of right side from torsion spring
extern float TdR_InitValue;              // Nm, Auxiliary parameter for right Td
extern float Estimated_FcR;              // N,  Cable force feedback of right side from load cell
extern float FcR_InitValue;              // N, Auxiliary parameter for right cable forve

extern float CableTorqueL;               // Nm, Left torque feedback from cable force 
extern float CableTorqueR;               // Nm, Right torque feedback from cable force 
// Nm, Compact Td feedback of left side actuation system for closed-loop 
// control from torsion spring torque feedback and cable force feedback
extern float Feedback_TdL;               
// Nm, Compact Td feedback of right side actuation system for closed-loop 
// control from torsion spring torque feedback and cable force feedback
extern float Feedback_TdR;  

/* Parameters for phase index determination */
// Nm, Critical torque value for left actuation system
extern float Critical_TdL;               
// Nm, Critical torque value for left actuation system
extern float Critical_TdR;
// Auxiliary parameter for slope of phase index profile determination, 0~1
extern float PhaseIndexCo;
// 0~1, 0 for DD and 1 for SEA, operation index of the left actuation system
extern float phaseIndexL;                
// 0~1, 0 for DD and 1 for SEA, operation index of the right actuation system
extern float phaseIndexR; 

/**
 * Control parameter initialization for Low-level controller
 * Initial parameters including: 
 * Reference torque command and Intermediate quantities related to PWM command.
 * Intermediate value of Interative force and hip angle feedback; 
 * PID struct parameters (PID controller parameters);  
 * Here use increment PID algorithm: 
 * Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
 */
void Control_Init(void);

/**
 * Pre-processing for sensor feedback to make sure 
 * the initial status of sensor is good for calibration
 */
void LLPreproSensorInit(void);

/**
 * Set the yaw angle of human trunk to zero
 * @param unsigned char - Yaw init mode: 1-force to set, other number-logic set
 * @param IMUAlo - IMU operation algorithm
 */
void yawAngleR20(uint8_t yawInitmode, IMUAlo aloMode);

/**
 * Yaw angle processing for practical control 
 */
void TrunkYawAngPro(void);

/**
 * Update operation status of the actuation system
 * @param float - Present Td feedback
 * @param float - Critical torque value
 * @param float - Slope determination parameter (0~1)
 * @return float - phase index (1 for SEA, 0 for DD)
 */
float PhaseIndexUpdate(float FeedbackTd, float CriticalTd, float SlopeK);

/**
 * Update the sin value of the angle between cable and human back 
 * for calculation of assistive torque feedback from cable force for left side
 * @return float - the angle between cable and human back of left side
 */
float sinofangleBetweenCableHB_L(void);

/**
 * Update the sin value of the angle between cable and human back 
 * for calculation of assistive torque feedback from cable force for right side
 * @return float - the angle between cable and human back of right side
 */
float sinofangleBetweenCableHB_R(void);

/**
 * Processing sensor feedback for closed-loop control and data sending to PC
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
