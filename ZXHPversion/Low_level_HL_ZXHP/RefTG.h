/***********************************************************************
 * Reference torque generation strategy
 **********************************************************************/

#ifndef __RefTG__H
#define __RefTG__H

#include "Control.h"
#include "FSM.h"

extern float desiredTorqueL;    // desired torque command of left motor actuation 
extern float desiredTorqueR;    // desired torque command of right motor actuation
extern float PredesiredTorqueL; // previous desired torque command of left motor actuation 
extern float PredesiredTorqueR; // previous desired torque command of right motor actuation 

/* 
 * Controller parameter for Reference Torque Generation strategy, 
 * the content may be adjusted according to future practical applied 
 * high-level strategy.
*/
typedef struct {
  // Parameters for RTG strategy
  float TrunkMass;         // kg, Subject's trunk mass
  float TrunkHalfLength;   // m, Half of subject's trunk length 
  float GComRatio;         // Gravity compensation ratio
  float ImpeKp;            // Nm/deg, Rendered stiffness of Impedance strategy
  float ImpeKv;            // Nm*s/deg, Rendered damping of Impedance strategy
} RTGCont;
extern RTGCont RTG_Subject1;   // RTG strategy parameters for specific subjects

/**
 * Control parameter initialization for RTG strategy
 * Initial controller including: 
 * Reference torque, strategy parameters and auxiliary parameters used for RTG strategy
 */
void RTG_Init(void);

/**
 * Reference torque generation 
 * @param unsigned char - control mode for torque generation strategy: 1-xx strategy, 2-xx strategy
 */
void HL_ReferTorqueGenerate(uint8_t RTGMode);

/**
 * Reference torque generation - Impedance strategy
 * @param float - Rendered stiffness (Nm/deg)
 * @param float - Rendered damping   (Nm/(deg/s))
 * @param float - Initial Position   (Angle)
 * @param float - Initial Velocity
 * @param float - Present Position   (Angle)
 * @param float - Present Velocity
 * @return float - Reference Torque
 */
float ImpedanceStra(float RendKp, float RendKv, float InitPos, float InitVel, float PrePos, float PreVel);

/**
 * Reference torque generation - Gravity Compensation strategy
 * @param float - Trunk mass
 * @param float - Half of Trunk Length
 * @param float - Initial Position (Angle)
 * @param float - Present Position (Angle)
 * @param float - Assistive Ratio
 * @return float - Reference Torque
 */
float GraCompenStra(float TMass, float HaTLength, float InitPos, float PrePos, float AsRatio);




#endif
