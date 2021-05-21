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
bool Control_update = true;    // control update flag



/*************************************** Intermediate auxiliary parameters for control ****************************/ 
// Parameters for lowe-level control
float Estimated_TdMotorCurrentL;   // Td feedback from left motor current feedback
float Estimated_TdMotorCurrentR;   // Td feedback from right motor current feedback
float Estimated_TdForceSensorL;    // Td feedback from left force sensor
float ForceSensorL_InitValue;      // Auxiliary parameter for left force sensor
float Estimated_TdForceSensorR;    // Td feedback from right force sensor
float ForceSensorR_InitValue;      // Auxiliary parameter for right force sensor
float Estimated_TdL;               // Estimated compact Td feedback of left side
float Estimated_TdR;               // Estimated compact Td feedback of right side

/**
 * Control parameter initialization for Low-level controller
 * Initial parameters including: 
 * PWM commands; Iterative force feedback; PID struct parameters (PID controller parameters) related to PWM command.
 * Here use increment PID algorithm: Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
 */
void Control_Init(void) {
  /* Initialize PWM command */
  PWM_commandL = 0;
  PWM_commandR = 0;
  
  /* Initialize interation torque feedback */
  Estimated_TdMotorCurrentL = 0;
  Estimated_TdMotorCurrentR = 0;
  Estimated_TdForceSensorL = 0;
  ForceSensorL_InitValue = 0;
  Estimated_TdForceSensorR = 0;
  ForceSensorR_InitValue = 0;
  Estimated_TdL = 0;
  Estimated_TdR = 0;

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
}

/**
 * Pre-processing for sensor feedback related to low-level controller 
 * to make sure the initial status of sensor is good for calibration
 */
void LLPreproSensorInit() {
  int8_t SensorReady;
  int8_t SensorReady_1;
  int8_t SensorReady_2;
  SensorReady = 0;
  while(SensorReady == 0){
    // Collect info from ADC including: ForceSensors(Interaction force) and Motor status 
    getADCaverage(1);
    delay(1);
    // Initialize the inital value for each sensor feedback
    // Notice to check the Initial value is ADC raw data or Processed data
    ForceSensorL_InitValue = Aver_ADC_value[ForceSensorL]/ForceSensorL_Sensitivity;
    ForceSensorR_InitValue = Aver_ADC_value[ForceSensorR]/ForceSensorR_Sensitivity;
    // Here place program to check if these initial value of each sensor is near the expected position. 
    // If not, recalibration the initial value of the sensor feedback 
    if(ForceSensorL_InitValue > ForceSensorL_CaliValue + ForceSensorL_Tol || ForceSensorL_InitValue < ForceSensorL_CaliValue - ForceSensorL_Tol)
    {SensorReady_1 = 0;}  
    else {SensorReady_1 = 1;}
    if(ForceSensorR_InitValue > ForceSensorR_CaliValue + ForceSensorR_Tol || ForceSensorR_InitValue < ForceSensorR_CaliValue - ForceSensorR_Tol)
    {SensorReady_2 = 0;}
    else {SensorReady_2 = 1;}
    SensorReady = SensorReady_1*SensorReady_2;
  }
  Serial.println("Sensor Ready for Low-level Controller.");  
}

/**
 * Processing sensor feedback for Low-level closed-loop control
 */
void sensorFeedbackPro(void) {
  // ------- Interation torque feedback info processing for low-level controller ------------------
//  Estimated_TdMotorCurrentL = (Aver_ADC_value[MotorCurrL]-2)*9/2;                    // Td feedback from motor driver, here ESCON set 0~4V:-9~9A
//  Estimated_TdForceSensorL = Aver_ADC_value[ForceSensorL]*ForceSensorL_Sensitivity;  // Td feedback from Force sensor L
//  Estimated_TdL = 0.1*Estimated_TdForceSensorL+0.9*Estimated_TdMotorCurrentL;        // Td feedback used for low-level closed-loop control
  
  // --------- Trunk flexion velocity info processing for high-level controller ------------ 

  // --------- Hip joint angle feedback info processing for high-level controller ----------
//  HipAngleL = (Aver_ADC_value[PotentioLP1]-HipAngleL_InitValue)/PotentioLP1_Sensitivity;

  // --------- Thigh angle feedback info processing for high-level controller --------------
//  ThighAngL = 180-HipAngL-angleActualC[rollChan];  //(TrunkFleAng = angleActualC[rollChan])
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
    pidL.currT = Estimated_TdL;    // get current toruqe feedback
    pidL.Err = pidL.set - pidL.currT;             // calculate the error of this time
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
    pidR.currT = Estimated_TdR;   // get current toruqe feedback
    pidR.Err = pidR.set - pidR.currT;            // calculate the error of this time
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
