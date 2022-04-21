/***********************************************************************
 * The PID control configuration and processing function &
 * System model parameters
 **********************************************************************/

#include "Control.h"

/**************************************** Low level PID control parameters definition ********************************/
PID pidL;  // control parameter of left motor
PID pidR;  // control parameter of right motor
// the desired torque from PC is defined in communication receiving parameter part
int16_t PWM_commandL;   // range: 0.1*PWMperiod_L~0.9*PWMperiod_L
int16_t PWM_commandR;   // range: 0.1*PWMperiod_R~0.9*PWMperiod_R
int8_t PWMSignL;        // to mark the rotation direction of the left motor
int8_t PWMSignR;        // to mark the rotation direction of the right motor
float maxInterTorqueL;  // restriction of the maximum allowable assistive torque of left hip considering safety and torque sensor feedback
float minInterTorqueL;  // restriction of the minimum allowable assistive torque of left hip considering safety and torque sensor feedback
float maxInterTorqueR;  // restriction of the maximum allowable assistive torque of right hip considering safety and torque sensor feedback
float minInterTorqueR;  // restriction of the minimum allowable assistive torque of right hip considering safety and torque sensor feedback
bool Control_update = true;    // control update flag

/* Parameters for human motion compensation */
bool MotionComEnable = true;
float lastHuMComL;
float humanMotionComL;
float deltaHuMComL;
float lastHuMComR;
float humanMotionComR;
float deltaHuMComR;


/*************************************** Intermediate auxiliary parameters for control ****************************/ 
// Parameters for lowe-level control
float Estimated_TdMotorCurrentL;   // Td feedback from left motor current feedback
float Estimated_TdMotorCurrentR;   // Td feedback from right motor current feedback
float Estimated_TdTorqueL;         // Td feedback from left torque sensor
float TorqueL_InitValue;           // Auxiliary parameter for left torque sensor
float Estimated_TdTorqueR;         // Td feedback from right torque sensor
float TorqueR_InitValue;           // Auxiliary parameter for right torque sensor
float Estimated_TdForceSensorL;    // Td feedback from left force sensor
float ForceSensorL_InitValue;      // Auxiliary parameter for left force sensor
float Estimated_TdForceSensorR;    // Td feedback from right force sensor
float ForceSensorR_InitValue;      // Auxiliary parameter for right force sensor
float Estimated_TdL;               // Estimated compact Td feedback of left side
float Estimated_TdR;               // Estimated compact Td feedback of right side

/**
 * Human motion effect compensation term
 * Here the system model parameters of the left&right CSEA system is assumed as the same
 */
void humanMotionCompen() {
  float CompenHipVelL;
  float CompenHipVelR;
  float CompenHipAccL;
  float CompenHipAccR;
  
  CompenHipVelL = -HipAngVelL;
  CompenHipVelR = -HipAngVelR;
  CompenHipAccL = -HipAngAccL;
  CompenHipAccR = -HipAngAccR; 

  lastHuMComL = humanMotionComL;
  lastHuMComR = humanMotionComR;

  // if(mode == Lowering) {
  //   // Ta = Tpid + (1/1.5 - 1)*Tr
  //   humanMotionComL = -0.33*desiredTorqueL;
  //   humanMotionComR = -0.33*desiredTorqueR;
  // }
  // else if(mode == Lifting) {
  //   // Ta = Tpid + (1/0.5 - 1)*Tr
  //   humanMotionComL = 0.45*desiredTorqueL;
  //   humanMotionComR = 0.45*desiredTorqueR;    
  // }
  // else if(mode == Walking) {
  //   humanMotionComL = -actuationJa*CompenHipAccL-actuationBa*CompenHipVelL;
  //   humanMotionComR = -actuationJa*CompenHipAccR-actuationBa*CompenHipVelR;
  // }
  // else {
  //   humanMotionComL = 0;
  //   humanMotionComR = 0;
  // }

  if(mode == Lowering || mode == Lifting || mode == Grasping) {
    humanMotionComL = actuationJa*CompenHipAccL+actuationBa*CompenHipVelL;
    humanMotionComR = actuationJa*CompenHipAccR+actuationBa*CompenHipVelR;
  }
  else if(mode == Walking) {
    humanMotionComL = actuationJa*CompenHipAccL+actuationBa*CompenHipVelL;
    humanMotionComR = actuationJa*CompenHipAccR+actuationBa*CompenHipVelR;  
  }
  else {
    humanMotionComL = 0;
    humanMotionComR = 0;
  } 
  
  // limitation of delta_compensation 
  deltaHuMComL = humanMotionComL - lastHuMComL;
  deltaHuMComR = humanMotionComR - lastHuMComR;
  if(Value_sign(deltaHuMComL)*deltaHuMComL >= deltaComLimit) {
    deltaHuMComL = Value_sign(deltaHuMComL)*deltaComLimit;
    humanMotionComL = lastHuMComL+deltaHuMComL;
  }
  if(Value_sign(deltaHuMComR)*deltaHuMComR >= deltaComLimit) {
    deltaHuMComR = Value_sign(deltaHuMComR)*deltaComLimit;
    humanMotionComR = lastHuMComR+deltaHuMComR;
  }  

}


/**
 * Control parameter initialization for Low-level controller
 * Initial parameters including: 
 * PWM commands; PID struct parameters (PID controller parameters) related to PWM command.
 * Here use increment PID algorithm: Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
 */
void Control_Init(void) {
  /* Initialize PWM command */
  PWM_commandL = 0;
  PWM_commandR = 0;
  PWMSignL = PosSign;
  PWMSignR = PosSign;  
  /* Initialize PID struct parameters*/
  // initialize the control parameter of left motor
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

  // initialize the control parameter of right motor
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

  /* Parameters for human motion compensation */
  lastHuMComL = 0;
  humanMotionComL = 0;
  deltaHuMComL = humanMotionComL - lastHuMComL;
  lastHuMComR = 0;
  humanMotionComR = 0;
  deltaHuMComR = humanMotionComR - lastHuMComR;

  /* Restriction of maximum and minimum interaction torque */
  maxInterTorqueL = 0.98*(0-TorqueSensorL_Offset)*TorqueSensorL_Sensitivity;    // restriction of the maximum allowable assistive torque of left hip considering safety and torque sensor feedback
  minInterTorqueL = 0.98*(2.5-TorqueSensorL_Offset)*TorqueSensorL_Sensitivity;  // restriction of the minimum allowable assistive torque of left hip considering safety and torque sensor feedback
  maxInterTorqueR = 0.98*(2.5-TorqueSensorR_Offset)*TorqueSensorR_Sensitivity;  // restriction of the maximum allowable assistive torque of right hip considering safety and torque sensor feedback
  minInterTorqueR = 0.98*(0-TorqueSensorR_Offset)*TorqueSensorR_Sensitivity;    // restriction of the minimum allowable assistive torque of right hip considering safety and torque sensor feedback
}

/**
 * Control auxiliary parameter initialization
 * Initial parameters including: 
 * Interative force feedback
 */
void ControlAux_Init() {
  /* Initialize interation torque feedback */
  Estimated_TdMotorCurrentL = 0;
  Estimated_TdMotorCurrentR = 0;
  Estimated_TdTorqueL = 0;         // Td feedback from left torque sensor
  TorqueL_InitValue = 0;           // Auxiliary parameter for left torque sensor
  Estimated_TdTorqueR = 0;         // Td feedback from right torque sensor
  TorqueR_InitValue = 0;           // Auxiliary parameter for right torque sensor
  Estimated_TdForceSensorL = 0;
  ForceSensorL_InitValue = 0;
  Estimated_TdForceSensorR = 0;
  ForceSensorR_InitValue = 0;
  Estimated_TdL = 0;
  Estimated_TdR = 0;  
}

/**
 * Pre-processing for sensor feedback related to low-level controller 
 * to make sure the initial status of sensor is good for calibration
 * @return int8_t - Sensor ready flag: 0-Not Ready; 1-Ready
 */
int8_t LLPreproSensorInit() {
  int8_t SensorReady;
  int8_t SensorReady_1;
  int8_t SensorReady_2;
  int8_t SensorReady_3;
  int8_t SensorReady_4;
    
  SensorReady = 0;
  // Collect info from ADC including: Potentiometets(HipAng), TorqueSensors/ForceSensors(Interaction force)
  // and Motor status
  getADCaverage(1);
  delay(1);
  // Initialize the inital value for each sensor feedback
  // Notice to check the Initial value is ADC raw data or Processed data
  TorqueL_InitValue = (Aver_ADC_value[TorqueSensorL]-TorqueSensorL_Offset)*TorqueSensorL_Sensitivity;
  TorqueR_InitValue = (Aver_ADC_value[TorqueSensorR]-TorqueSensorR_Offset)*TorqueSensorR_Sensitivity;
  ForceSensorL_InitValue = Aver_ADC_value[ForceSensorL]/ForceSensorL_Sensitivity;
  ForceSensorR_InitValue = Aver_ADC_value[ForceSensorR]/ForceSensorR_Sensitivity;
  // Here place program to check if these initial value of each sensor is near the expected position. 
  // If not, recalibration the initial value of the sensor feedback 
  if(TorqueL_InitValue > TorqueSensorL_CaliValue + TorqueSensorL_Tol || TorqueL_InitValue < TorqueSensorL_CaliValue - TorqueSensorL_Tol)
  {SensorReady_1 = 0;}
  else {SensorReady_1 = 1;}
  if(TorqueR_InitValue > TorqueSensorR_CaliValue + TorqueSensorR_Tol || TorqueR_InitValue < TorqueSensorR_CaliValue - TorqueSensorR_Tol)
  {SensorReady_2 = 0;}
  else {SensorReady_2 = 1;}
  if(ForceSensorL_InitValue > ForceSensorL_CaliValue + ForceSensorL_Tol || ForceSensorL_InitValue < ForceSensorL_CaliValue - ForceSensorL_Tol)
  {SensorReady_3 = 0;}  
  else {SensorReady_3 = 1;}
  if(ForceSensorR_InitValue > ForceSensorR_CaliValue + ForceSensorR_Tol || ForceSensorR_InitValue < ForceSensorR_CaliValue - ForceSensorR_Tol)
  {SensorReady_4 = 0;}
  else {SensorReady_4 = 1;}
  // This is because NO FORCE SENSOR is used at present
  SensorReady_3 = 1;  
  SensorReady_4 = 1;
  SensorReady = SensorReady_1*SensorReady_2*SensorReady_3*SensorReady_4;

  /* This is for sensor initial value checking */
//  Serial.print("TL")
//  Serial.print(TorqueL_InitValue);
//  Serial.print("TR")
//  Serial.print(TorqueR_InitValue);
  SensorReady = 1;

  if(SensorReady == 0) {
    Serial.println("Sensor NotReady for Low-level Controller.");
  }
  else {
    Serial.println("Sensor Ready for Low-level Controller.");
  }
  return SensorReady;
}

/**
 * Pre-processing for sensor feedback related to both low-level controller 
 * and high-level control to make sure the initial status of sensor is good for calibration
 * @return int8_t - Sensor ready flag: 0-Not Ready; 1-Ready
 */
int8_t PreproSensorInit() {
  int8_t SensorReady;
  int8_t SensorReady_1;
  int8_t SensorReady_2;

  SensorReady = 0;
  SensorReady_1 = LLPreproSensorInit();
  SensorReady_2 = HLPreproSensorInit();
  SensorReady = SensorReady_1*SensorReady_2;

  if(SensorReady == 0) {
    Serial.println("NotReady.");
  }
  else {
    Serial.println("Ready.");
  }
  return SensorReady;
}

/**
 * Processing sensor feedback for Low-level closed-loop control
 */
void sensorFeedbackPro() {
  MovingAverageFilter(TorqueSensorL,3);
  MovingAverageFilter(TorqueSensorR,3);
  /* Interation torque feedback info processing for low-level controller */
  // Td feedback from motor driver, here ESCON set 0~4V:-7~7A
//  Estimated_TdMotorCurrentL = (Aver_ADC_value[MotorCurrL]-2)*7/2;
//  Estimated_TdMotorCurrentR = (Aver_ADC_value[MotorCurrR]-2)*7/2;
  // Td feedback from Torque sensor                                            
  Estimated_TdTorqueL = (Aver_ADC_value[TorqueSensorL]-TorqueSensorL_Offset)*TorqueSensorL_Sensitivity - TorqueL_InitValue;
  Estimated_TdTorqueR = (Aver_ADC_value[TorqueSensorR]-TorqueSensorR_Offset)*TorqueSensorR_Sensitivity - TorqueR_InitValue;
  // Td feedback from Force sensor
//  Estimated_TdForceSensorL = Aver_ADC_value[ForceSensorL]/ForceSensorL_Sensitivity - ForceSensorL_Sensitivity;  
//  Estimated_TdForceSensorR = Aver_ADC_value[ForceSensorR]/ForceSensorR_Sensitivity - ForceSensorR_Sensitivity; 
  // Td feedback used for low-level closed-loop control
  Estimated_TdL = Estimated_TdTorqueL;        
  Estimated_TdR = Estimated_TdTorqueR;

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
    pidL.currT = Estimated_TdL;              // get current toruqe feedback
    pidL.Err = pidL.set - pidL.currT;        // calculate the error of this time
    // P
    dk1L = pidL.Err - pidL.Err_p;
    PoutL = pidL.Kp*dk1L;
    // Adjust mechanism of P components
//    if(Value_sign(PoutL)*PoutL > LimitDelta_KPL) {
//      PoutL = Value_sign(PoutL)*LimitDelta_KPL;
//      pidL.Err = PoutL/pidL.Kp+pidL.Err_p;
//    }
    // I
    IoutL = (pidL.Kp*pidL.Tcontrol)/pidL.Ti;
    IoutL = IoutL*pidL.Err*0;
    // D
    dk2L = pidL.Err+pidL.Err_pp-2*pidL.Err_p;
    DoutL = (pidL.Kp*pidL.Td)/pidL.Tcontrol;
    DoutL = DoutL*dk2L;
    // calculate the delta value of this time
    pidL.Delta_Ta = (PoutL+IoutL+DoutL)+deltaHuMComL;
    /* set limitation of sudden variation of control output */
    // make sure the interaction torque feedback are between the minmum and maximum restriction
    if(Estimated_TdL >= maxInterTorqueL-TorqueL_InitValue && pidL.Delta_Ta > 0) {
      pidL.Delta_Ta = 0;
    }
    else if(Estimated_TdL <= minInterTorqueL-TorqueL_InitValue && pidL.Delta_Ta < 0) {
      pidL.Delta_Ta = 0;
    }
    // limited delta Ta
    if((Value_sign(pidL.Delta_Ta)*pidL.Delta_Ta) >= LimitDelta_TaL) {
    	pidL.Delta_Ta = LimitDelta_TaL*Value_sign(pidL.Delta_Ta);
    }
    pidL.currTa += pidL.Delta_Ta;
    /* set limitation of total controller output */
    // Avoid sudden motor reversion under small torque command with small torque feedback due to stiction for safety
    // by limit total controller command output
    // Better restrict for different status like TM or AM with different constraints like average human motion compensation value and a small value
    if((Value_sign(Estimated_TdL)*Estimated_TdL) <= 2 && (Value_sign(desiredTorqueL)*desiredTorqueL) <= 2) {
      if((Value_sign(pidL.currTa)*pidL.currTa) >= 4) {
        pidL.currTa = 4*Value_sign(pidL.currTa);
      }
    }
    // Bounded control output
    if((Value_sign(pidL.currTa)*pidL.currTa) >= LimitTotal_TaL) {
    	pidL.currTa = LimitTotal_TaL*Value_sign(pidL.currTa);
    }
    // determine motor rotation direction
    if(pidL.currTa >= 0) {
      PWMSignL = PosSign;
      digitalWrite(MotorRotationL,LOW);
    }
    else if(pidL.currTa < 0) {
      PWMSignL = NegSign;
      digitalWrite(MotorRotationL,HIGH);
    }
    
    pidL.currCurrent = Value_sign(pidL.currTa)*pidL.currTa/GearRatio/MotorCurrentConstant;
    pidL.currpwm = pidL.pwm_cycle*(pidL.currCurrent*0.8/MotorMaximumCurrent+0.1);
    // set limitation of PWM duty cycle
    if(pidL.currpwm > PWMUpperBound*pidL.pwm_cycle) {
      pidL.currpwm = PWMUpperBound*pidL.pwm_cycle;
    }
    else if(pidL.currpwm < PWMLowerBound*pidL.pwm_cycle) {
      pidL.currpwm = PWMLowerBound*pidL.pwm_cycle;
    }
    PWM_commandL = pidL.currpwm;
    //update the error
    pidL.Err_pp = pidL.Err_p;
    pidL.Err_p = pidL.Err;

    /************************ PID control for right motor *************************/
    pidR.set = desiredTorqueR;
    pidR.currT = Estimated_TdR;   // get current toruqe feedback
    pidR.Err = pidR.set - pidR.currT;            // calculate the error of this time
    // P
    dk1R = pidR.Err - pidR.Err_p;
    PoutR = pidR.Kp*dk1R;
    // Adjust mechanism of P components
//    if(Value_sign(PoutR)*PoutR > LimitDelta_KPR) {
//      PoutR = Value_sign(PoutR)*LimitDelta_KPR;
//      pidR.Err = PoutR/pidR.Kp+pidR.Err_p;
//    }
    // I
    IoutR = (pidR.Kp*pidR.Tcontrol)/pidR.Ti;
    IoutR = IoutR*pidR.Err*0;
    // D
    dk2R = pidR.Err+pidR.Err_pp-2*pidR.Err_p;
    DoutR = (pidR.Kp*pidR.Td)/pidR.Tcontrol;
    DoutR = DoutR*dk2R;
    // calculate the delta value of this time
    pidR.Delta_Ta = (PoutR+IoutR+DoutR)+deltaHuMComR;
    /* set limitation of sudden variation of control output */
    // make sure the interaction torque feedback are between the minmum and maximum restriction
    if(Estimated_TdR >= maxInterTorqueR-TorqueR_InitValue && pidR.Delta_Ta > 0) {
      pidR.Delta_Ta = 0;
    }
    else if(Estimated_TdR <= minInterTorqueR-TorqueR_InitValue && pidR.Delta_Ta < 0) {
      pidR.Delta_Ta = 0;
    }
    // limited delta Ta
    if((Value_sign(pidR.Delta_Ta)*pidR.Delta_Ta) >= LimitDelta_TaR) {
    	pidR.Delta_Ta = LimitDelta_TaR*Value_sign(pidR.Delta_Ta);
    }
    pidR.currTa += pidR.Delta_Ta;
    /* set limitation of total controller output */
    // Avoid sudden motor reversion under small torque command with small torque feedback due to stiction for safety
    // by limit total controller command output
    // Better restrict for different status like TM or AM with different constraints like average human motion compensation value and a small value
    if((Value_sign(Estimated_TdR)*Estimated_TdR) <= 2 && (Value_sign(desiredTorqueR)*desiredTorqueR) <= 2) {
      if((Value_sign(pidR.currTa)*pidR.currTa) >= 4) {
        pidR.currTa = 4*Value_sign(pidR.currTa);
      }
    }
    // Bounded control output
    if((Value_sign(pidR.currTa)*pidR.currTa) >= LimitTotal_TaR) {
      pidR.currTa = LimitTotal_TaR*Value_sign(pidR.currTa);
    }
    // determine motor rotation direction
    if(pidR.currTa >= 0) {
      PWMSignR = PosSign;
      digitalWrite(MotorRotationR,HIGH);
    }
    else if(pidR.currTa < 0) {
      PWMSignR = NegSign;
      digitalWrite(MotorRotationR,LOW);
    }
        
    pidR.currCurrent = Value_sign(pidR.currTa)*pidR.currTa/GearRatio/MotorCurrentConstant;
    pidR.currpwm = pidR.pwm_cycle*(pidR.currCurrent*0.8/MotorMaximumCurrent+0.1);
    // set limitation of PWM duty cycle
    if(pidR.currpwm > PWMUpperBound*pidR.pwm_cycle) {
      pidR.currpwm = PWMUpperBound*pidR.pwm_cycle;
    }
    else if(pidR.currpwm < PWMLowerBound*pidR.pwm_cycle) {
      pidR.currpwm = PWMLowerBound*pidR.pwm_cycle;
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
    PWM_commandR = PWMperiod_R*(desiredCurrentR*0.8/MotorMaximumCurrent+0.1);
    if(PWM_commandL >= PWMUpperBound*PWMperiod_L) {
      PWM_commandL = PWMUpperBound*PWMperiod_L;
    }
    else if(PWM_commandL <= PWMLowerBound*PWMperiod_L) {
      PWM_commandL = PWMLowerBound*PWMperiod_L;
    }
    if(PWM_commandR >= PWMUpperBound*PWMperiod_R) {
      PWM_commandR = PWMUpperBound*PWMperiod_R;
    }
    else if(PWM_commandR <= PWMLowerBound*PWMperiod_R) {
      PWM_commandR = PWMLowerBound*PWMperiod_R;
    }
  }
  /* Set the PWM duty cycle */
  // In stop and exit mode the PWM command is forced to set as PWMLowerBound*PWMperiod
  if(mode == ExitState) {
    PWM_commandL = PWMLowerBound*PWMperiod_L;
    PWM_commandR = PWMLowerBound*PWMperiod_R;
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
