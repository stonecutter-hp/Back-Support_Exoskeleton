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
  RTG_Subject1.GComRatio = 0.2;        // Gravity compensation ratio
  RTG_Subject1.TrunkMass = 40;         // kg, Subject's trunk mass
  RTG_Subject1.TrunkHalfLength = 0.295;// m, Half of subject's trunk length
  RTG_Subject1.ImpeKp = 0.2;           // Nm/deg, Rendered stiffness of Impedance strategy
  RTG_Subject1.ImpeKv = 0;             // Nm*s/deg, Rendered damping of Impedance strategy   
  
}

/**
 * Reference torque generation 
 * @param unsigned char - control mode for torque generation strategy: 1-xx strategy, 2-xx strategy
 */
void HL_ReferTorqueGenerate(uint8_t RTGMode) {
  float InterTorque;
  InterTorque = 0.0;
  // User reference torque generation strategy v1 (Referring to the 'Thoughts Keeping notbook')
  if(RTGMode == 1) {
    if(mode == Standing) {
      desiredTorqueL = 0;
      desiredTorqueR = 0;
      // enable motor control
      digitalWrite(MotorEnableL,HIGH);
      digitalWrite(MotorEnableR,HIGH);
    }
    else if(mode == Walking) {
      desiredTorqueL = 0;
      desiredTorqueR = 0;
      // disable motor control
      digitalWrite(MotorEnableL,LOW);
      digitalWrite(MotorEnableR,LOW);      
    }
    else if(mode == ExitState) {
      desiredTorqueL = 0;
      desiredTorqueR = 0;
      // Reset controller
      Control_Init();
      // disable motor control
      digitalWrite(MotorEnableL,LOW);
      digitalWrite(MotorEnableR,LOW);       
    }
    else {
      // Stoop: Gravity compensation
      if(tech == Stoop) {
        desiredTorqueL = 0.5*GraCompenStra(RTG_Subject1.TrunkMass, RTG_Subject1.TrunkHalfLength, TrunkFleAng_T0InitValue, TrunkFleAng, RTG_Subject1.GComRatio);
        desiredTorqueR = desiredTorqueL;
      }
      // Squat: Impedance strategy
      else if(tech == Squat) {
        desiredTorqueL = ImpedanceStra(RTG_Subject1.ImpeKp, RTG_Subject1.ImpeKv, ThighAngL_T0InitValue, 0, ThighAngL, 0);
        desiredTorqueR = ImpedanceStra(RTG_Subject1.ImpeKp, RTG_Subject1.ImpeKv, ThighAngR_T0InitValue, 0, ThighAngR, 0);      
      }
      // Semi-Squat: Gravity compensation + Impedance strategy
      else {
        InterTorque = 0.5*GraCompenStra(RTG_Subject1.TrunkMass, RTG_Subject1.TrunkHalfLength, TrunkFleAng_T0InitValue, TrunkFleAng, RTG_Subject1.GComRatio);
        desiredTorqueL = InterTorque+ImpedanceStra(RTG_Subject1.ImpeKp, RTG_Subject1.ImpeKv, ThighAngL_T0InitValue, 0, ThighAngL, 0);
        desiredTorqueR = InterTorque+ImpedanceStra(RTG_Subject1.ImpeKp, RTG_Subject1.ImpeKv, ThighAngR_T0InitValue, 0, ThighAngR, 0);   
      }
      // enable motor control
      digitalWrite(MotorEnableL,HIGH);
      digitalWrite(MotorEnableR,HIGH);
    }
    
  }
  
}

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
float ImpedanceStra(float RendKp, float RendKv, float InitPos, float InitVel, float PrePos, float PreVel) {
  float ImpeTorque;
  if(PrePos < InitPos) {
    PrePos = InitPos;
  }
  // T = Kp*Delta_alpha+Kv*Delta_alpha_vel
  ImpeTorque = RendKp*(PrePos - InitPos)+RendKv*(PreVel - InitVel);
  return ImpeTorque;
}

/**
 * Reference torque generation - Gravity Compensation strategy
 * @param float - Trunk mass
 * @param float - Half of Trunk Length
 * @param float - Initial Position (Angle)
 * @param float - Present Position (Angle)
 * @param float - Assistive Ratio
 * @return float - Reference Torque
 */
float GraCompenStra(float TMass, float HaTLength, float InitPos, float PrePos, float AsRatio) {
  float GraCompTorque;
  if(PrePos < InitPos) {
    PrePos = InitPos;
  }
  // T = Ratio*m*g*sin(delta_alpha)*L
  GraCompTorque = AsRatio*TMass*9.8*sin((PrePos-InitPos)*d2r)*HaTLength;
}
