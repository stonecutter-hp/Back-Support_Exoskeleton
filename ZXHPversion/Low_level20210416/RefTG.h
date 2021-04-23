/***********************************************************************
 * Reference torque generation strategy
 **********************************************************************/

#ifndef __RefTG__H
#define __RefTG__H

#include "Control.h"
#include "FSM.h"

extern float desiredTorqueL;    // desired motor torque of left motor
extern float desiredTorqueR;    // desired motor torque of right motor

/* 
 * Controller parameter for Reference Torque Generation strategy, 
 * the content may be adjusted according to future practical applied 
 * high-level strategy.
*/
typedef struct {
  // Parameters for RTG strategy
  float TrunkMass;         // kg, Subject's trunk mass
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
 * @para unsigned char - control mode for torque generation strategy: 1-xx strategy, 2-xx strategy
 */
void HL_ReferTorqueGenerate(uint8_t RTGMode);









#endif
