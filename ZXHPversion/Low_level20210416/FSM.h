/***********************************************************************
 * The Finite State Machine Alogrithm
 **********************************************************************/

#ifndef __FSMachine_H__
#define __FSMachine_H__

#include "Control.h"

// Motion type define
typedef enum {
  StopState = 0,
  Standing = 1,
  Walking = 2,
  Lowering = 3,
  Grasping = 4,
  Lifting = 5,
  ExitState = 6,
} MotionType;
extern MotionType mode;         // this time's motion mode
extern MotionType PreMode;      // last time's motion mode
// tech type: 1-Stoop bending; 2-Squat bending; 3-Semi-squat bending
extern uint8_t tech;            // bending tech flag
// 1-left; 2-right; 0-none
extern uint8_t side;            // Asymmetric side flag
// Controller parameter and threshold for high-level control strategy, the content may be adjusted according to future
// practical applied high-level strategy
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
  // Parameters for RTG strategy
  float TrunkMass;         // kg, Subject's trunk mass
  float ImpeKp;            // Nm/deg, Rendered stiffness of Impedance strategy
  float ImpeKv;            // Nm*s/deg, Rendered damping of Impedance strategy
}HLCont;  // Controller parameter and threshold for high-level control strategy
extern HLCont Subject1;    // Parameters for specified subjects




#endif
