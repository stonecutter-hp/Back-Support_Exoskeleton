/***********************************************************************
 * The Finite State Machine Alogrithm for User Intention Detection
 **********************************************************************/

#ifndef __FSMachine_H__
#define __FSMachine_H__

#include "Control.h"
#include "IMU.h"
#include "ADC.h"
#include "RefTG.h"

/* High-level controller program running info */
extern bool HLControl_update;   // high-level control update flag
extern float HLUpdateFre;       // hihg-level control frequency

/* Motion type define */
typedef enum {
  StopState = 0,
  Standing = 1,
  Walking = 2,
  Lowering = 3,
  Grasping = 4,
  Lifting = 5,
  ExitState = 6
} MotionType;

/* Asymmetric bending side type define */
typedef enum {
  none = 0,
  left = 1,
  right = 2
} AsymSide;

/* Bending techniques type define */
typedef enum {
  Stoop = 0,
  Squat = 1,
  SemiSquat = 2
} BendTech;

/* High-level control status flag */ 
extern MotionType mode;         // this time's motion mode flag
extern MotionType PreMode;      // last time's motion mode flag
extern AsymSide side;           // Asymmetric side flag
extern BendTech tech;           // bending tech flag    
      
/* 
 * Controller parameter and threshold for User Intention Detection strategy, 
 * the content may be adjusted according to future practical applied 
 * high-level strategy.
*/
typedef struct {
  // Parameters for UID strategy
  float ThrTrunkFleAng;    // Threshold for trunk flexion angle
  float ThrTrunkFleVel;    // Threshold for trunk flexion velocity
  float ThrHipAngMean;     // Threshold for mean value of summation of left and right hip angle  
  float ThrHipAngDiff;     // Threshold for difference between left and right hip angle
  float ThrHipAngStd;      // Threshold for standard deviation of mean hip angle
  float ThrHipVel;         // Threshold for mean hip angle velocity
  // Notice that ThrThighAngMean should not be larger than ThrHipAngMean - ThrTrunkFleAng if ThighAng comes from
  // calculation of (HipAng - TrunkFleAng) instead of measurement
  float ThrThighAngMean;   // Threshold for mean value of summation of left and right thigh angle
  float ThrThighAngStd;    // Threshold for standard deviation of mean thigh angle
  float ThrThighAngVel;    // Threshold for mean thigh angle velocity
  float ThrHipAngDiffStd;  // Threshold for standard deviation of difference between left and right hip angle
  // Standard deviation calculation range
  int StdRange;
}UIDCont;                  // Controller parameter and threshold for high-level control strategy
extern UIDCont UID_Subject1;    // UID strategy parameters for specific subjects

/* High-level controller related sensor feedback calibration value definition */
// Expected initial value range (CaliValue +- Tol) of sensor feedback for initial calibration
// the initial values should be adjusted along with prototype design
#define HipAngL_CaliValue 0
#define HipAngL_Tol 0
#define HipAngR_CaliValue 0
#define HipAngR_Tol 0
#define TrunkFleAng_CaliValue 0
#define TrunkFleAng_Tol 0
#define TrunkFleYaw_CaliValue 0
#define TrunkFleYaw_Tol 0
#define ForcedInit 1
#define LogicInit  0

/* Intermediate auxiliary parameters for UID strategy */
// Parameters Directly feedback from sensor
extern float HipAngL;                     // Left hip joint angle
extern float HipAngL_InitValue;           // Auxiliary parameter for left hip joint angle
extern float HipAngR;                     // Right hip joint angle
extern float HipAngR_InitValue;           // Auxiliary parameter for right hip joint angle
extern float TrunkYawAng;                 // Trunk yaw angle
extern float TrunkYaw_InitValue;          // Auxiliary parameter for trunk yaw angle
extern float TrunkFleAng;                 // Trunk flexion angle
extern float TrunkFleAng_InitValue;       // Auxiliary parameter for trunk pitch angle
extern float TrunkFleVel;                 // Trunk flexion angular velocity
// Parameters calculated from sensor feedback
extern float HipAngMean;                  // (Left hip angle + right hip angle)/2
extern float HipAngDiff;                  // (Left hip angle - right hip angle)
extern float HipAngStd;                   // Std(HipAngMean) within certain time range
extern float HipAngVel;                   // Velocity of HipAngMean
extern float ThighAngL;                   // Left thigh angle
extern float ThighAngR;                   // Right thigh angle
extern float ThighAngMean;                // (Left thigh angle + right thigh angle)/2
extern float ThighAng_InitValue;          // Auxiliary parameter for thigh angle
extern float ThighAngStd;                 // Std(ThighAngMean) within certain time range
extern float HipAngDiffStd;               // Std(HipAngDiff) within certain time range
// A window store the historical HipAngMean value of certain cycle for standard deviation calculation
extern float HipAngMeanPre[FilterCycles];
extern float HipAngMeanBar;               // Auxiliary parameter X_bar for standard deviation calculation
// A window store the historical HipAngDiff value of certain cycle for standard deviation calculation
extern float HipAngDiffPre[FilterCycles];
extern float HipAngDiffBar;               // Auxiliary parameter X_bar for standard deviation calculation

/**
 * Control parameter initialization for UID strategy
 * Initial controller including: 
 * Sensor feedbacks, status flags, thresholds and auxiliary parameters used for UID strategy
 */
void UID_Init(void);

/**
 * Pre-processing for sensor feedback related to high-level controller 
 * to make sure the initial status of sensor is good for calibration
 */
void HLPreproSensorInit(void);

/**
 * Set the yaw angle of human trunk to zero
 * @param unsigned char - Yaw init mode: 1-force to set, other number-logic set
 * @param IMUAlo - IMU operation algorithm
 */
void yawAngleR20(uint8_t yawInitmode, IMUAlo aloMode);

/**
 * Processing sensor feedback for High-level closed-loop control and data sending
 */
void HLsensorFeedbackPro(void);

/**
 * Yaw angle processing for practical control
 */
void TrunkYawAngPro(void);

/**
 * Calculate standard deviation for HipAngMean within certain cycles
 * @param int - cycles: 1~FilterCycles
 * @param return - calculated standard deviation
 */
double HipAngStdCal(int cycles);

/**
 * Calculate standard deviation for HipAngDiff within certain cycles
 * @param int - cycles: 1~FilterCycles
 * @param return - calculated standard deviation
 */
double HipAngDiffStdCal(int cycles);

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
 * @para unsigned char - control version for user intenntion detection: 1-v1 algorithm, 2-v2 algorithm
 */
void HL_UserIntentDetect(uint8_t UIDMode);


#endif
