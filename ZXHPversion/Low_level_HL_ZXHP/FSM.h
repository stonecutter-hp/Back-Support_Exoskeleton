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
extern float HLUpdateFre;       // high-level control frequency

/* Motion type define */
typedef enum {
  ExitState = 1,
  Standing = 2,
  Walking = 3,
  Lowering = 4,
  Grasping = 5,
  Lifting = 6
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
extern bool YawAngleUpdate;     // Yaw angle reset flag
extern bool RecordStateReset;   // States recorded at T0 and the peak moment reset flag
extern bool BendTechClassFlag;  // Bending tech clasiffication flag
      
/* 
 * Controller parameter and threshold for User Intention Detection strategy, 
 * the content may be adjusted according to future practical applied 
 * high-level strategy.
*/
typedef struct {
  /*Parameters for UID strategy */
  float ThrHipAngMean_1;     // Threshold 1 for mean value of summation of left and right hip angle
  float ThrHipAngMean_2;     // Threshold 2 for mean value of summation of left and right hip angle
  float ThrHipAngMean_3;     // Threshold 3 for mean value of summation of left and right hip angle 

  float ThrHipAngDiff_1;     // Threshold 1 for difference between left and right hip angle

  float ThrHipVel_1;         // deg/s, Threshold 1 for mean hip angle velocity
  float ThrHipVel_2;         // deg/s, Threshold 2 for mean hip angle velocity
  float ThrHipVel_3;         // deg/s, Threshold 3 for mean hip angle velocity
  float ThrHipVel_4;         // deg/s, Threshold 4 for mean hip angle velocity

  float ThrTrunkFleAng_1;    // Threshold 1 for trunk flexion angle
  float ThrTrunkFleAng_2;    // Threshold 2 for trunk flexion angle
  float ThrThighAngMean_1;   // Threshold 1 for mean thigh angle
  float ThrThighAngMean_2;   // Threshold 2 for mean thigh angle
  float ThrRatioTech_1;      // ratio_1 for bending tech classification
  float ThrRatioTech_2;      // ratio_1 for bending tech classification

  // Standard deviation calculation range
  int StdRange;

  // Ratio tolerance related to hip angle for transition between standing and lowering&lifting 
  float RatioTol_1;
  float RatioTol_2;
  float RatioTol_3;

  float ThrHipAngDiffVel_1;  // deg/s, Threshold 1 for difference of angular velocity between left and right hip angle
  float ThrHipAngDiffVel_2;  // deg/s, Threshold 2 for difference of angular velocity between left and right hip angle

  float ThrTrunkFleVel;      // Threshold for trunk flexion velocity
  float ThrThighAngStd;      // Threshold for standard deviation of mean thigh angle
  float ThrThighAngVel;      // Threshold for mean thigh angle velocity
  float ThrHipAngStd;        // Threshold for standard deviation of mean hip angle

  /* Thresholds for Exit Phase */ 
  float ThrHipAngDiffVel_3;  // deg/s, Threshold 2 for difference of angular velocity between left and right hip angle
  float ThrTrunkFleAngEMin;  // Threshold for allowable minimum trunk flexion angle
  float ThrTrunkFleAngEMax;  // Threshold for allowable maximum trunk flexion angle
  float ThrThighAngMeanEMin; // Threshold for allowable minimum thigh flexion angle
  float ThrThighAngMeanEMax; // Threshold for allowable maximum thigh flexion angle
}UIDCont;                    // Controller parameter and threshold for high-level control strategy
extern UIDCont UID_Subject1;    // UID strategy parameters for specific subjects

/* High-level controller related sensor feedback calibration value definition */
// Expected initial value range (CaliValue +- Tol) of sensor feedback for initial calibration
// the initial values should be adjusted along with prototype design
#define HipAngL_CaliValue 41
#define HipAngL_Tol 10
#define HipAngR_CaliValue 214
#define HipAngR_Tol 10
#define TrunkFleAng_CaliValue 0
#define TrunkFleAng_Tol 10
#define TrunkFleYaw_CaliValue 0
#define TrunkFleYaw_Tol 1000
#define ForcedInit 1
#define LogicInit  0

/* Intermediate auxiliary parameters for UID strategy */
// Parameters Directly feedback from sensor
extern float HipAngL;                     // Left hip joint angle
extern float HipVelL_Motor;               // Left hip joint velocity from motor velocity feedback
extern float HipAngL_InitValue;           // Auxiliary parameter for left hip joint angle
extern float HipAngL_T0InitValue;         // Auxiliary parameter of T0 left hip joint angle
extern float HipAngL_MaxValue;            // Auxiliary parameter of Max left hip joint bending angle
extern float HipAngR;                     // Right hip joint angle
extern float HipVelR_Motor;               // Right hip joint velocity from motor velocity feedback
extern float HipAngR_InitValue;           // Auxiliary parameter for right hip joint angle
extern float HipAngR_T0InitValue;         // Auxiliary parameter of T0 right hip joint angle
extern float HipAngR_MaxValue;            // Auxiliary parameter of Max right hip joint bending angle
extern float TrunkYawAng;                 // Trunk yaw angle
extern float TrunkYaw_InitValue;          // Auxiliary parameter for trunk yaw angle
extern float TrunkYaw_T0InitValue;        // Auxiliary parameter for T0 trunk yaw angle
extern float TrunkFleAng;                 // Trunk flexion angle
extern float TrunkFleAng_MaxValue;        // Auxiliary parameter of Max trunk pitch angle
extern float TrunkFleAng_InitValue;       // Auxiliary parameter for trunk pitch angle
extern float TrunkFleAng_T0InitValue;     // Auxiliary parameter for T0 trunk pitch angle
extern float PreTrunkVel;                 // Last time's Trunk flexion angular velocity (For acceleration calculation)
extern float TrunkFleVel;                 // Trunk flexion angular velocity
extern float PreTrunkFleAcc;              // Last time's Trunk flexion angular acceleration (For filter)
extern float TrunkFleAcc;                 // Trunk flexion angular acceleration

// Parameters calculated from sensor feedback
extern float HipAngMean;                  // (Left hip angle + right hip angle)/2
extern float HipAngDiff;                  // (Left hip angle - right hip angle)
extern float HipAngStd;                   // Std(HipAngMean) within certain time range
extern float HipAngStdSign;               // sign of Std(HipAngMean)
extern float PreHipAngVelL;               // Last time's velocity of HipAngL
extern float HipAngVelL;                  // Velocity of HipAngL
extern float PreHipAngAccL;               // Last time's Acceleration of HipAngL (For filter)
extern float HipAngAccL;                  // Acceleration of HipAngL
extern float PreHipAngVelR;               // Last time's velocity of HipAngR
extern float HipAngVelR;                  // Velocity of HipAngR
extern float PreHipAngAccR;               // Last time's Acceleration of HipAngR (For filter)
extern float HipAngAccR;                  // Acceleration of HipAngR
extern float HipAngVel;                   // (HipAngVelL + HipAngVelR)/2
extern float ThighAngL;                   // Left thigh angle
extern float ThighAngL_T0InitValue;       // Auxiliary parameter of T0 left thigh angle for RTG
extern float ThighAngR;                   // Right thigh angle
extern float ThighAngR_T0InitValue;       // Auxiliary parameter of T0 right thigh angle for RTG
extern float ThighAngMean;                // (Left thigh angle + right thigh angle)/2
extern float ThighAngStd;                 // Std(ThighAngMean) within certain time range
extern float HipAngDiffStd;               // Std(HipAngDiff) within certain time range
extern float HipAngDiffVel;               // abs(HipAngVelL - HipAngVelR)
// Time parameters for velocity calculation
extern unsigned long starttime;          
extern unsigned long stoptime;
extern unsigned long looptime;
// A window store the historical HipAngMean value of certain cycle for standard deviation calculation
extern float HipAngPreL[FilterCycles];
extern float HipAngPreR[FilterCycles];
extern float HipAngMeanPre[FilterCycles];
extern float HipAngMeanBar;               // Auxiliary parameter X_bar for standard deviation calculation
// A window store the historical HipAngDiff value and its abs value of certain cycle for standard deviation calculation
extern float HipAngDiffPre[FilterCycles];
extern float AbsHipAngDiffPre[FilterCycles];
extern float HipAngDiffBar;               // Auxiliary parameter X_bar for standard deviation calculation
// A window store the historical HipAngStd value of certain cycle for Finite state machine
extern float HipAngStdPre[FilterCycles];
// A window store the historical abs(HipAngVel) value of certain cycle for Finite state machine
extern float HipAngVelPre[FilterCycles];
// A window store the historical HipAngDiffStd value of certain cycle for Finite state machine
extern float HipAngDiffStdPre[FilterCycles];
// A window store the historical TrunkFleAng value of certain cycle for Finite state machine
extern float TrunkFleAngPre[FilterCycles];

/**
 * Control parameter initialization for UID strategy
 * Initial controller including: 
 * Sensor feedbacks, status flags, thresholds and auxiliary parameters used for UID strategy
 */
void UID_Init(void);

/**
 * Pre-processing for sensor feedback related to high-level controller 
 * to make sure the initial status of sensor is good for calibration
 * @return int8_t - Sensor ready flag: 0-Not Ready; 1-Ready
 */
int8_t HLPreproSensorInit(void);

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
 * Here the population standard deviation is calculated
 * @param int - cycles: 1~FilterCycles
 * @return double - calculated standard deviation
 */
double HipAngStdCal(int cycles);

/**
 * Calculate standard deviation for HipAngDiff within certain cycles
 * Here the population standard deviation is calculated
 * @param int - cycles: 1~FilterCycles
 * @return double - calculated standard deviation
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

/**************** Functional function of each phase for the Finite-state machine ******************/
/**
 * System operation during standing phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void StandingPhase(void);

/**
 * System operation during Walking phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void WalkingPhase(void);

/**
 * System operation during Lowering phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void LoweringPhase(void);

/**
 * System operation during Grasping phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void GraspingPhase(void);

/**
 * System operation during Lifting phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void LiftingPhase(void);

/**
 * System operation during Exit phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void ExitPhase(void);

/**
 * Bending technique classification
 * @param int - number of continous requirment satisfied items at once: 1~FilterCycles 
 */
void BendTechClassify(int Techcycles);

/**
 * Functional funciton to check continuously threshold requirement is meeted
 * @param float - threshold
 * @param float[] - The array to be checked
 * @param int - number of continous requirment satisfied items at once: 1~FilterCycles  
 * @param unsigned char - threshold requirements: 0- <Threshold, 1- >Threshold
 * @return bool - true: satisfied; false: not satisfied
 */
bool ConThresReqCheck(float threshold, float *value, int cycles, uint8_t ThreRequire);

/**
 * Functional funciton to detect peak angle
 * @param float[] - The array to be detected  
 * @param unsigned char - peak value mode: 0-detect minimum value, 1-detect maximum value
 * @return float - detected peakvalue
 */
float PeakvalueDetect(float *Pevalue, uint8_t Peakmode);

/**
 * Functional function to detect the direction of angle variation
 * Notice this function usually be called after variation is exceed certain threshold
 * @param float[] - The array to be detected 
 * @param int - interval between process involved data point
 * @return unsigned char - detected direction: 1-increasing direction, 2-decreasing direction, 0-uncertain
 */
uint8_t DireStdDetect(float *Angvalue, int cycles); 

/**
 * Low-pass filter 
 * @param float - cut off frequency of the low-pass filter
 * @param float - sampling rate
 * @return float - coefficient of the filter 
 */
float lowPassFilter(float cutFre, float samplRate);

#endif
