/***********************************************************************
 * Reference torque generation strategy
 **********************************************************************/

#include "RefTG.h"

float desiredTorqueL;    // desired motor torque of left motor
float desiredTorqueR;    // desired motor torque of right motor

RTGCont RTG_Subject1;    // RTG strategy parameters for specific subjects

/**
 * Control parameter initialization for RTG strategy
 * Initial controller including: 
 * Reference torque, strategy parameters and auxiliary parameters used for RTG strategy
 */
void RTG_Init(void) {
  /* Initialize reference torque */
  desiredTorqueL = 0;
  desiredTorqueR = 0;
  
  /* Initialized strategy parameters */
  RTG_Subject1.TrunkMass = 40;         // kg, Subject's trunk mass
  RTG_Subject1.ImpeKp = 0.2;           // Nm/deg, Rendered stiffness of Impedance strategy
  RTG_Subject1.ImpeKv = 0;             // Nm*s/deg, Rendered damping of Impedance strategy   
}

/**
 * Reference torque generation 
 * @para unsigned char - control mode for torque generation strategy: 1-xx strategy, 2-xx strategy
 */
void HL_ReferTorqueGenerate(uint8_t RTGMode) {
  // Here != 100 is a illustration of mode selection
  if(RTGMode != 100) {
    if(mode = Standing) {
      desiredTorqueL = 0;
      desiredTorqueR = 0;
    }
  // ----- Here replace for detailed user intention detection alogrithm ------- 
  }
}
