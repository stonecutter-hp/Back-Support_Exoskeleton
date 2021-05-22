/***********************************************************************
 * The PID control configuration and processing function 
 * System model parameters
 **********************************************************************/

#include "Control.h"

PID pidL;  // control parameter of left motor
PID pidR;  // control parameter of right motor
// the desired torque from PC is defined in communication receiving parameter part
int16_t PWM_commandL;   // range: 0.1*PWMperiod_L~0.9*PWMperiod_L
int16_t PWM_commandR;   // range: 0.1*PWMperiod_R~0.9*PWMperiod_R
bool Control_update = true;  // control update flag

/* Intermediate auxiliary parameters directly from sensor processing for controller feedback */
float HipAngL;                    // deg, Left hip joint angle
float HipAngL_InitValue;          // deg, Auxiliary parameter for left hip joint angle
float Estimated_TdL;              // Nm, Td feedback of left side from torsion spring
float TdL_InitValue;              // Nm, Auxiliary parameter for left Td
float Estimated_FcL;              // N,  Cable force feedback of left side from load cell
float FcL_InitValue;              // N, Auxiliary parameter for left cable forve

float HipAngR;                    // deg, Right hip joint angle
float HipAngR_InitValue;          // deg, Auxiliary parameter for right hip joint angle
float Estimated_TdR;              // Nm, Td feedback of right side from torsion spring
float TdR_InitValue;              // Nm, Auxiliary parameter for right Td
float Estimated_FcR;              // N,  Cable force feedback of right side from load cell
float FcR_InitValue;              // N, Auxiliary parameter for right cable forve

float CableTorqueL;               // Nm, Left torque feedback from cable force 
float CableTorqueR;               // Nm, Right torque feedback from cable force
// Nm, Compact Td feedback of left side actuation system for closed-loop 
// control from torsion spring torque feedback and cable force feedback
float Feedback_TdL;               
// Nm, Compact Td feedback of right side actuation system for closed-loop 
// control from torsion spring torque feedback and cable force feedback
float Feedback_TdR;  

/* Parameters for phase index determination */
// Nm, Critical torque value for left actuation system
float Critical_TdL;               
// Nm, Critical torque value for left actuation system
float Critical_TdR;
// Auxiliary parameter for slope of phase index profile determination, 0~1
float PhaseIndexCo;
// 0~1, 0 for DD and 1 for SEA, operation index of the left actuation system
float phaseIndexL;                
// 0~1, 0 for DD and 1 for SEA, operation index of the right actuation system
float phaseIndexR;

/**
 * Control parameter initialization for Low-level controller
 * Initial parameters including: 
 * Reference torque command and Intermediate quantities related to PWM command.
 * Intermediate value of Interative force and hip angle feedback; 
 * PID struct parameters (PID controller parameters);  
 * Here use increment PID algorithm: 
 * Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
 */
void Control_Init(void) {
  /* Initialize PWM command and reference torque command */
  PWM_commandL = 0;
  PWM_commandR = 0;
  desiredTorqueL = 0;
  desiredTorqueR = 0;

  /* Initialize mode and side from high-level UID strategy */
  // Motion detection mode, default is 0 (stop state) to send Ready signal for 
  // for handshake with high-level controller
  mode = 0;   
  PreMode = mode;
  side = 0;   // Asymmetric side, default is 0 (no asymmetric)

  /* Initialize intermediate value of sensor feedback */
  HipAngL = 0;
  HipAngL_InitValue = 0;
  Estimated_TdL = 0;
  TdL_InitValue = 0;
  Estimated_FcL = 0;
  FcL_InitValue = 0;

  HipAngR = 0;
  HipAngR_InitValue = 0;
  Estimated_TdR = 0;
  TdR_InitValue = 0;
  Estimated_FcR = 0;
  FcR_InitValue = 0;

  CableTorqueL = 0;
  CableTorqueR = 0;
  Feedback_TdL = 0;
  Feedback_TdR = 0;
  
  /* Parameters for phase index determination */
  // Nm, Critical torque value for left actuation system
  Critical_TdL = TorsionStiffnessL*MechConsAngleL;               
  // Nm, Critical torque value for left actuation system
  Critical_TdR = TorsionStiffnessR*MechConsAngleR;
  // Auxiliary parameter for slope of phase index profile determination, 0~1
  PhaseIndexCo = 0.2;
  // 0~1, 0 for DD and 1 for SEA, operation index of the left actuation system
  phaseIndexL = 1;  // initial as SEA status                
  // 0~1, 0 for DD and 1 for SEA, operation index of the right actuation system
  phaseIndexR = 1;  // initial as SEA status

  /* Initialize the control parameter of left motor */
  pidL.set = desiredTorqueL;
  pidL.currTa = 0;
  pidL.currCurrent = pidL.currTa/MotorCurrentConstant;
  pidL.pwm_cycle = PWMperiod_L;
  pidL.currpwm = pidL.pwm_cycle*(pidL.currCurrent*0.8/MotorMaximumCurrent+0.1);     
  pidL.Tcontrol = TIM3_OverflowValue;  // corresopnding to timer3, unit:us while prescale coefficient = 72
  pidL.Kp = KP_L;                      // should be adjusted
  pidL.Td = pidL.Tcontrol*KD_L/KP_L;   // should be adjusted, unit:us
  pidL.Ti = pidL.Tcontrol*KP_L/KI_L;   // should be adjusted, unit:us
  
  pidL.Err = 0;
  pidL.Err_p = 0;
  pidL.Err_pp = 0;

  /* initialize the control parameter of right motor */
  pidR.set = desiredTorqueR;
  pidR.currTa = 0;
  pidR.currCurrent = pidR.currTa/MotorCurrentConstant;
  pidR.pwm_cycle = PWMperiod_R;
  pidR.currpwm = pidR.pwm_cycle*(pidR.currCurrent*0.8/MotorMaximumCurrent+0.1);     
  pidR.Tcontrol = TIM3_OverflowValue;  // corresopnding to timer3, unit:us while prescale coefficient = 72
  pidR.Kp = KP_R;                      // should be adjusted
  pidR.Td = pidR.Tcontrol*KD_R/KP_R;   // should be adjusted, unit:us
  pidR.Ti = pidR.Tcontrol*KP_R/KI_R;   // should be adjusted, unit:us
    
  pidR.Err = 0;
  pidR.Err_p = 0;
  pidR.Err_pp = 0;
}

/**
 * Pre-processing for sensor feedback to make sure 
 * the initial status of sensor is good for calibration
 */
void LLPreproSensorInit() {
  int8_t SensorReady;
  int8_t SensorReady_TdL;
  int8_t SensorReady_TdR;
  int8_t SensorReady_HipAngL;
  int8_t SensorReady_HipAngR;
  int8_t SensorReady_FcL;
  int8_t SensorReady_FcR;  
  int8_t SensorReady_FlxAng;

  SensorReady = 0;
  while(SensorReady == 0){
    // Collect info from ADC including: Hip angle, Cable force, Torsion spring torque and Motor status 
    getADCaverage(1);
    delay(1);
    // Initialize present yaw angle as 0 reference. Notice inside the function info will be collected
    // from IMUC simultaneously including: TrunkAng, TrunkVel
    yawAngleR20(ForcedInit,OperaitonAloIMUC);
    delay(1);
    // Initialize the inital value for each sensor feedback
    // Notice to check the Initial value is ADC raw data or Processed data
    TdL_InitValue = Aver_ADC_value[PotentioLP1]/PotentioLP1_Sensitivity*TorsionStiffnessL;
    TdR_InitValue = Aver_ADC_value[PotentioRP3]/PotentioLP3_Sensitivity*TorsionStiffnessR;
    HipAngL_InitValue = Aver_ADC_value[PotentioLP2]/PotentioLP2_Sensitivity;
    HipAngR_InitValue = Aver_ADC_value[PotentioRP4]/PotentioLP4_Sensitivity;
    FcL_InitValue = (Aver_ADC_value[LoadCellL]-1.25)/LoadCellL_Sensitivity;
    FcR_InitValue = (Aver_ADC_value[LoadCellR]-1.25)/LoadCellR_Sensitivity;
    TrunkFleAng_InitValue = angleActualC[rollChan];
    // Here place program to check if these initial value of each sensor is near the expected position. 
    // If not, recalibration the initial value of the sensor feedback 
    if(TdL_InitValue > TdL_CaliValue + TdL_Tol || TdL_InitValue < TdL_CaliValue - TdL_Tol)
    {SensorReady_TdL = 0;}  
    else {SensorReady_TdL = 1;}
    if(TdR_InitValue > TdR_CaliValue + TdR_Tol || TdR_InitValue < TdR_CaliValue - TdR_Tol)
    {SensorReady_TdR = 0;}
    else {SensorReady_TdR = 1;}
    if(HipAngL_InitValue > HipAngL_CaliValue + HipAngL_Tol || HipAngL_InitValue < HipAngL_CaliValue - HipAngL_Tol)
    {SensorReady_HipAngL = 0;}
    else {SensorReady_HipAngL = 1;}
    if(HipAngR_InitValue > HipAngR_CaliValue + HipAngR_Tol || HipAngR_InitValue < HipAngR_CaliValue - HipAngR_Tol)
    {SensorReady_HipAngR = 0;}
    else {SensorReady_HipAngR = 1;}
    if(FcL_InitValue > FcL_CaliValue + FcL_Tol || FcL_InitValue < FcL_CaliValue - FcL_Tol)
    {SensorReady_FcL = 0;}  
    else {SensorReady_FcL = 1;}
    if(FcR_InitValue > FcR_CaliValue + FcR_Tol || FcR_InitValue < FcR_CaliValue - FcR_Tol)
    {SensorReady_FcR = 0;}  
    else {SensorReady_FcR = 1;}    
    if(TrunkFleAng_InitValue > TrunkFleAng_CaliValue + TrunkFleAng_Tol || TrunkFleAng_InitValue < TrunkFleAng_CaliValue - TrunkFleAng_Tol)
    {SensorReady_FlxAng = 0;}  
    else {SensorReady_FlxAng = 1;} 

    SensorReady = SensorReady_TdL*SensorReady_TdR*SensorReady_HipAngL*SensorReady_HipAngR;
    SensorReady = SensorReady*SensorReady_FcL*SensorReady_FcR*SensorReady_FlxAng;
  }
  Serial.println("Ready.");  
}

/**
 * Set the yaw angle of human trunk to zero
 * @param unsigned char - Yaw init mode: 1-force to set, other number-logic set
 * @param IMUAlo - IMU operation algorithm
 */
void yawAngleR20(uint8_t yawInitmode, IMUAlo aloMode){
  // Roughly yaw angle return to zero logic: 
  // Detect mode 1 and premode is larger than 3
  // i.e., From bending back to other motion
  if(yawInitmode == ForcedInit) {
    if(aloMode == IMU9Axis) {
      getIMUangleT();
      TrunkYaw_InitValue = angleActualC[yawChan];
    }
    else if(aloMode == IMU6Axis) {
      // set2zeroL();
      // set2zeroR();
      set2zeroT();
      getIMUangleT();
      TrunkYaw_InitValue = 0;
    }
  }
  else {
    if(mode == 0 && PreMode > 3) {
      if(aloMode == IMU9Axis) {
        getIMUangleT();
        TrunkYaw_InitValue = angleActualC[yawChan];
      }
      else if(aloMode == IMU6Axis) {
        // set2zeroL();
        // set2zeroR();
        set2zeroT();
        getIMUangleT();
        TrunkYaw_InitValue = 0;        
      }
    }
  }
}

/**
 * Yaw angle processing for practical control 
 */
void TrunkYawAngPro() {
  // Trunk yaw angle feedback info procesisng for high-level controller
  TrunkYawAng = angleActualC[yawChan] - TrunkYaw_InitValue;
  if(TrunkYawAng > 180) {
    TrunkYawAng = TrunkYawAng-360;
  }
  else if(TrunkYawAng < -180) {
    TrunkYawAng = TrunkYawAng+360;
  }
}


/**
 * Update operation status of the actuation system
 * @param float - Present Td feedback
 * @param float - Critical torque value
 * @param float - Slope determination parameter (0~1)
 * @return float - phase index (1 for SEA, 0 for DD)
 */
float PhaseIndexUpdate(float FeedbackTd, float CriticalTd, float SlopeK) {
  float PhaseIndex;
  double intervalue1;
  double intervalue2;
  intervalue1 = pow(min(0.0,(double) (FeedbackTd*FeedbackTd-CriticalTd*CriticalTd)),2);
  intervalue2 = pow(pow(SlopeK*CriticalTd,2) - pow(CriticalTd,2),2);
  PhaseIndex = 1-pow(min(0.0,intervalue1-intervalue2),2)/pow(intervalue2,2);
  
  return PhaseIndex;
}

/**
 * Update the sin value of the angle between cable and human back 
 * for calculation of assistive torque feedback from cable force for left side
 * @return float - the angle between cable and human back of left side
 */
float sinofangleBetweenCableHB_L() {
  float sinofAngleCableHB;
  float cableLength;
  float AngleSupportBeamHB;

  // Calculate the angle between support beam and human back (deg)
  AngleSupportBeamHB = TrunkFleAng + Theta0_L - Estimated_TdL/TorsionStiffnessL;
  // Calculate the cable length through law of cosines
  cableLength = sqrt(pow(SupportBeamLengthL,2)+pow(HumanBackLength,2)-2*SupportBeamLengthL*HumanBackLength*cos(AngleSupportBeamHB*d2r));
  // Calculate the sin value through law of sines
  sinofAngleCableHB = SupportBeamLengthL*sin(AngleSupportBeamHB*d2r)/cableLength;

  return sinofAngleCableHB;
}

/**
 * Update the sin value of the angle between cable and human back 
 * for calculation of assistive torque feedback from cable force for right side
 * @return float - the angle between cable and human back of right side
 */
float sinofangleBetweenCableHB_R() {
  float sinofAngleCableHB;
  float cableLength;
  float AngleSupportBeamHB;

  // Calculate the angle between support beam and human back (deg)
  AngleSupportBeamHB = TrunkFleAng + Theta0_R - Estimated_TdR/TorsionStiffnessR;
  // Calculate the cable length through law of cosines
  cableLength = sqrt(pow(SupportBeamLengthR,2)+pow(HumanBackLength,2)-2*SupportBeamLengthR*HumanBackLength*cos(AngleSupportBeamHB*d2r));
  // Calculate the sin value through law of sines
  sinofAngleCableHB = SupportBeamLengthR*sin(AngleSupportBeamHB*d2r)/cableLength;

  return sinofAngleCableHB;
}

/**
 * Processing sensor feedback for closed-loop control and data sending to PC
 */
void sensorFeedbackPro(void) {
  /* Motor status processing for stop mode */
  // if high-level command stop state/exit unit/error state exceed certain number
  if(mode < 2 || mode > 7) {
  	// Set zero for reference torque
  	desiredTorqueR = 0;
  	desiredTorqueL = 0;
  	// Disable motor control
  	digitalWrite(MotorEnableL,LOW);
    digitalWrite(MotorEnableR,LOW);
  }
  else {
    // Enable motor control
    digitalWrite(MotorEnableL,HIGH); 	
    digitalWrite(MotorEnableR,HIGH);
  }

  /* Interation torque feedback info processing */
  // Estimated_ImuAssistiveTorqueL = (angleActualA[0]-SupportBeamAngleL_InitValue)*TorsionStiffnessL;
  Estimated_TdL = Aver_ADC_value[PotentioLP1]/PotentioLP1_Sensitivity*TorsionStiffnessL - TdL_InitValue; 
  if(Estimated_TdL < 0)  {Estimated_TdL = 0;}
  Estimated_TdR = Aver_ADC_value[PotentioRP3]/PotentioLP3_Sensitivity*TorsionStiffnessR - TdR_InitValue; 
  if(Estimated_TdR < 0)  {Estimated_TdR = 0;}
  
  /* Cable force feedback info processing */
  Estimated_FcL = (Aver_ADC_value[LoadCellL]-1.25)/LoadCellL_Sensitivity - FcL_InitValue; 
  if(Estimated_FcL < 0)  {Estimated_FcL = 0;}
  Estimated_FcR = (Aver_ADC_value[LoadCellR]-1.25)/LoadCellR_Sensitivity - FcR_InitValue; 
  if(Estimated_FcR < 0)  {Estimated_FcR = 0;}

  /* Hip angle info processing */
  HipAngL = Aver_ADC_value[PotentioLP2]/PotentioLP2_Sensitivity - HipAngL_InitValue;
  HipAngR = Aver_ADC_value[PotentioRP4]/PotentioLP4_Sensitivity - HipAngR_InitValue;

  /* Trunk yaw and pitch angle & pitch velocity feedback info procesisng */
  TrunkYawAngPro();
  TrunkFleAng = angleActualC[rollChan] - TrunkFleAng_InitValue;
  TrunkFleVel = velActualC[rollChan];

  /* Compact torque feedback calculation for low-level feedback controller */
  // Calculate the torque from cable force; here if..else... is to reduce 
  // the calculation burden
  if(phaseIndexL < 1) {
    CableTorqueL = Estimated_FcL*HumanBackLength*sinofangleBetweenCableHB_L();
  }
  else {
    CableTorqueL = 0;
  }
  if(phaseIndexR < 1) {
   CableTorqueR = Estimated_FcR*HumanBackLength*sinofangleBetweenCableHB_R();
  }
  else {
    CableTorqueR = 0;
  }
  /* Notice here update phasindex after torque feedback update so that if cable torque
     is larger than practical torque. For example, practical SEA status < Critical Td 
     with DD phase index. Then transition phase occurs with SEA dynamics as expected
  */
  // Update compact assistive torque feedback
  Feedback_TdL = phaseIndexL*Estimated_TdL + (1-phaseIndexL)*CableTorqueL;
  Feedback_TdR = phaseIndexR*Estimated_TdR + (1-phaseIndexR)*CableTorqueR;
  // Update actuation system operation status index
  phaseIndexL = PhaseIndexUpdate(Feedback_TdL, Critical_TdL, PhaseIndexCo);
  phaseIndexR = PhaseIndexUpdate(Feedback_TdR, Critical_TdR, PhaseIndexCo);

  // Estimated_PoAssistiveTorqueR = (Aver_ADC_value[PotentioLP2]-PotentioRP1_InitValue)/PotentioRP1_Sensitivity*TorsionStiffnessR; 
  // Aver_ADC_value[MotorCurrL] =  (Aver_ADC_value[MotorCurrL]-2)*9/2;   // here ESCON set 0~4V:-9~9A
  // Aver_ADC_value[MotorVeloL] = (Aver_ADC_value[MotorVeloL]-2)*4000/2; // here ESCON set 0~4V:-4000~4000rpm


}

/**
 * Calculate control command (PWM duty cycle) accroding to command received from PC
 * and information received from ADC and IMU
 * @para unsigned char - control mode: 1-PID control, 2-Open loop control
 * Here use increment PID algorithm: Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
 */
void Control(uint8_t ContMode) {
  // for PID control
  float dk1L,dk2L;
  float PoutL,IoutL,DoutL;
  float dk1R,dk2R;
  float PoutR,IoutR,DoutR;

  // For open-loop control
  float desiredCurrentL;
  float desiredCurrentR;

  if(ContMode == 1) {
    /************************ PID control for left motor *************************/
    pidL.set = desiredTorqueL;
    pidL.currT = Feedback_TdL;                 // get current toruqe feedback
    pidL.Err = pidL.set - pidL.currT;          // calculate the error of this time
    // P
    dk1L = pidL.Err - pidL.Err_p;
    PoutL = pidL.Kp*dk1L;
    // I
    IoutL = (pidL.Kp*pidL.Tcontrol)/pidL.Ti;
    IoutL = IoutL*pidL.Err*0;
    // D
    dk2L = pidL.Err+pidL.Err_pp-2*pidL.Err_p;
    DoutL = (pidL.Kp*pidL.Td)/pidL.Tcontrol;
    DoutL = DoutL*dk2L;
    // calculate the delta value of this time
    pidL.Delta_Ta = (PoutL+IoutL+DoutL);
    // set limitation of surdden variation of control output
    if((Value_sign(pidL.Delta_Ta)*pidL.Delta_Ta) >= LimitDelta_TaL) {
    	pidL.Delta_Ta = LimitDelta_TaL*Value_sign(pidL.Delta_Ta);
    }
    pidL.currTa += pidL.Delta_Ta;
    // set limitation of total controller output
    if((Value_sign(pidL.currTa)*pidL.currTa) >= LimitTotal_TaL) {
    	pidL.currTa = LimitTotal_TaL*Value_sign(pidL.currTa);
    }
    pidL.currCurrent = Value_sign(pidL.currTa)*pidL.currTa/GearRatio/MotorCurrentConstant;
    pidL.currpwm = pidL.pwm_cycle*(pidL.currCurrent*0.8/MotorMaximumCurrent+0.1);
    // set limitation of PWM duty cycle
    if(pidL.currpwm > 0.9*pidL.pwm_cycle) {
      pidL.currpwm = 0.9*pidL.pwm_cycle;
    }
    else if(pidL.currpwm < 0.1*pidL.pwm_cycle) {
      pidL.currpwm = 0.1*pidL.pwm_cycle;
    }
    // determine motor rotation direction
    if(pidL.currTa >= 0) {
    	digitalWrite(MotorRotationL,LOW);
    }
    else if(pidL.currTa < 0) {
    	digitalWrite(MotorRotationL,HIGH);
    }
    PWM_commandL = pidL.currpwm;
    //update the error
    pidL.Err_pp = pidL.Err_p;
    pidL.Err_p = pidL.Err;

    /************************ PID control for right motor *************************/
    pidR.set = desiredTorqueR;
    pidR.currT = Feedback_TdR;                // get current toruqe feedback
    pidR.Err = pidR.set - pidR.currT;         // calculate the error of this time
    // P
    dk1R = pidR.Err - pidR.Err_p;
    PoutR = pidR.Kp*dk1R;
    // I
    IoutR = (pidR.Kp*pidR.Tcontrol)/pidR.Ti;
    IoutR = IoutR*pidR.Err;
    // D
    dk2R = pidR.Err+pidR.Err_pp-2*pidR.Err_p;
    DoutR = (pidR.Kp*pidR.Td)/pidR.Tcontrol;
    DoutR = DoutR*dk2R;
    // calculate the delta value of this time
    pidR.Delta_Ta = (PoutR+IoutR+DoutR);
    // set limitation of surdden variation of control output
    if((Value_sign(pidR.Delta_Ta)*pidR.Delta_Ta) >= LimitDelta_TaR) {
    	pidR.Delta_Ta = LimitDelta_TaR*Value_sign(pidR.Delta_Ta);
    }
    pidR.currTa += pidR.Delta_Ta;
    // set limitation of total controller output
    if((Value_sign(pidR.currTa)*pidR.currTa) >= LimitTotal_TaR) {
    	pidR.currTa = LimitTotal_TaR*Value_sign(pidR.currTa);
    }
    pidR.currCurrent = Value_sign(pidR.currTa)*pidR.currTa/GearRatio/MotorCurrentConstant;
    pidR.currpwm = pidR.pwm_cycle*(pidR.currCurrent*0.8/MotorMaximumCurrent+0.1);
    // set limitation of PWM duty cycle
    if(pidR.currpwm > 0.9*pidR.pwm_cycle) {
      pidR.currpwm = 0.9*pidR.pwm_cycle;
    }
    else if(pidR.currpwm < 0.1*pidR.pwm_cycle) {
      pidR.currpwm = 0.1*pidR.pwm_cycle;
    }
    // determine motor rotation direction
    if(pidR.currTa >= 0) {
    	digitalWrite(MotorRotationR,LOW);
    }
    else if(pidR.currTa < 0) {
    	digitalWrite(MotorRotationR,HIGH);
    }
    PWM_commandR = pidR.currpwm;
    //update the error
    pidR.Err_pp = pidR.Err_p;
    pidR.Err_p = pidR.Err;
  }

  // Open-loop control
  else if(ContMode == 2) {
    desiredCurrentL = desiredTorqueL/GearRatio/MotorCurrentConstant;
    desiredCurrentR = desiredTorqueR/GearRatio/MotorCurrentConstant;
    PWM_commandL = PWMperiod_L*(desiredCurrentL*0.8/MotorMaximumCurrent+0.1);
    if(PWM_commandL >= 0.9*PWMperiod_L) {
      PWM_commandL = 0.9*PWMperiod_L;
    }
    else if(PWM_commandL <= 0.1*PWMperiod_L) {
      PWM_commandL = 0.1*PWMperiod_L;
    }
    if(PWM_commandR >= 0.9*PWMperiod_R) {
      PWM_commandR = 0.9*PWMperiod_R;
    }
    else if(PWM_commandR <= 0.1*PWMperiod_R) {
      PWM_commandR = 0.1*PWMperiod_R;
    }
  }
  // set the pwm duty cycle  
  MotorPWMoutput(PWM_commandL,PWM_commandR);        
}

/**
 * Set the pwm duty cycle for both of the motors
 * @param unsigned int - PWMcommandL/PWMcommandR: compared value
 *        range in 0~PWMperiod_L/PWMperiod_R
 */
void MotorPWMoutput(uint16_t PWMcommandL, uint16_t PWMcommandR) {
  Timer1.setCompare(TIM1_CH1,PWM_commandL);
  delay(1);
  Timer2.setCompare(TIM2_CH2,PWM_commandR);
}
