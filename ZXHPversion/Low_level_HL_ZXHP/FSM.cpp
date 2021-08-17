/***********************************************************************
 * The Finite State Machine Alogrithm for User Intention Detection
 **********************************************************************/

#include "FSM.h"


/* High-level controller program running info */
bool HLControl_update = true;  // high-level control update flag
float HLUpdateFre;             // high-level control frequency

/* Status flag for UID strategy */
MotionType mode;               // this time's motion mode flag
MotionType PreMode;            // last time's motion mode flag
AsymSide side;                 // Asymmetric side flag
BendTech tech;                 // bending tech flag  
bool YawAngleUpdate = true;    // Yaw angle reset complete flag
bool RecordStateReset = true;  // States recorded at T0 and the peak moment reset flag

/* Controller parameters and thresholds for UID strategy */
UIDCont UID_Subject1;          // UID strategy parameters for specific subjects

/* Intermediate auxiliary parameters for UID strategy */
// Parameters Directly feedback from sensor
float HipAngL;                     // Left hip joint angle
float HipVelL_Motor;               // Left hip joint velocity from motor velocity feedback
float HipAngL_InitValue;           // Auxiliary parameter for left hip joint angle
float HipAngL_T0InitValue;         // Auxiliary parameter of T0 left hip joint angle
float HipAngL_MaxValue;            // Auxiliary parameter of Max left hip joint bending angle
float HipAngR;                     // Right hip joint angle
float HipVelR_Motor;               // Right hip joint velocity from motor velocity feedback
float HipAngR_InitValue;           // Auxiliary parameter for right hip joint angle
float HipAngR_T0InitValue;         // Auxiliary parameter of T0 right hip joint angle
float HipAngR_MaxValue;            // Auxiliary parameter of Max right hip joint bending angle
float TrunkYawAng;                 // Trunk yaw angle
float TrunkYaw_InitValue;          // Auxiliary parameter for trunk yaw angle
float TrunkYaw_T0InitValue;        // Auxiliary parameter for T0 trunk yaw angle
float TrunkFleAng;                 // Trunk flexion angle
float TrunkFleAng_InitValue;       // Auxiliary parameter for trunk pitch angle
float TrunkFleAng_T0InitValue;     // Auxiliary parameter for T0 trunk pitch angle
float TrunkFleVel;                 // Trunk flexion angular velocity
// Parameters calculated from sensor feedback
float HipAngMean;                  // (Left hip angle + right hip angle)/2
float HipAngDiff;                  // (Left hip angle - right hip angle)
float HipAngStd;                   // Std(HipAngMean) within certain time range
float HipAngVelL;                  // Velocity of HipAngL
float HipAngVelR;                  // Velocity of HipAngR
float HipAngVel;                   // Velocity of HipAngMean
float ThighAngL;                   // Left thigh angle
float ThighAngL_T0InitValue;       // Auxiliary parameter of T0 left thigh angle for RTG
float ThighAngR;                   // Right thigh angle
float ThighAngR_T0InitValue;       // Auxiliary parameter of T0 right thigh angle for RTG
float ThighAngMean;                // (Left thigh angle + right thigh angle)/2
float ThighAngStd;                 // Std(ThighAngMean) within certain time range
float HipAngDiffStd;               // Std(HipAngDiff) within certain time range
// Time parameters for velocity calculation
unsigned long starttime;          
unsigned long stoptime;
unsigned long looptime;
// A window store the historical HipAngMean value of certain cycle for standard deviation calculation
float HipAngPreL[FilterCycles];
float HipAngPreR[FilterCycles];
float HipAngMeanPre[FilterCycles];
float HipAngMeanBar;               // Auxiliary parameter X_bar for standard deviation calculation
// A window store the historical HipAngDiff value of certain cycle for standard deviation calculation
float HipAngDiffPre[FilterCycles];
float HipAngDiffBar;               // Auxiliary parameter X_bar for standard deviation calculation
// A window store the historical HipAngStd value of certain cycle for Finite state machine
float HipAngStdPre[FilterCycles];
// A window store the historical HipAngDiffStd value of certain cycle for Finite state machine
float HipAngDiffStdPre[FilterCycles];

/**
 * Control parameter initialization for UID strategy
 * Initial controller including: 
 * Sensor feedbacks, status flags, thresholds and auxiliary parameters used for UID strategy
 */
void UID_Init(void) {
  /* Loop time Initialization */
  starttime = 1;
  stoptime = 2;
  looptime = stoptime - starttime;
  /* Initialize sensor feedbacks */
  // Parameters for UID strategy directly feedback from sensor
  HipAngL = 0;                     // Left hip joint angle
  HipVelL_Motor = 0;               // Left hip joint velocity from motor velocity feedback
  HipAngL_InitValue = 0;           // Auxiliary parameter for left hip joint angle
  HipAngL_T0InitValue = 0;         // Auxiliary parameter of T0 left hip joint angle
  HipAngL_MaxValue = 0;            // Auxiliary parameter of Max left hip joint bending angle
  HipAngR = 0;                     // Right hip joint angle
  HipVelR_Motor = 0;               // Right hip joint velocity from motor velocity feedback
  HipAngR_InitValue = 0;           // Auxiliary parameter for right hip joint angle
  HipAngR_T0InitValue = 0;         // Auxiliary parameter of T0 right hip joint angle
  HipAngR_MaxValue = 0;            // Auxiliary parameter of Max right hip joint bending angle
  TrunkYawAng = 0;                 // Trunk yaw angle
  TrunkYaw_InitValue = 0;          // Auxiliary parameter for trunk yaw angle
  TrunkYaw_T0InitValue = 0;        // Auxiliary parameter for T0 trunk yaw angle
  TrunkFleAng = 0;                 // Trunk flexion angle
  TrunkFleAng_InitValue = 0;       // Auxiliary parameter for trunk pitch angle
  TrunkFleAng_T0InitValue = 0;     // Auxiliary parameter for T0 trunk pitch angle
  TrunkFleVel = 0;                 // Trunk flexion angular velocity
  // Parameters for UID strategy calculated from sensor feedback
  HipAngMean = 0;                  // (Left hip angle + right hip angle)/2
  HipAngDiff = 0;                  // (Left hip angle - right hip angle)
  HipAngStd = 0;                   // Std(HipAngMean) within certain time range
  HipAngVelL = 0;                  // Velocity of HipAngL
  HipAngVelR = 0;                  // Velocity of HipAngR
  HipAngVel = 0;                   // Velocity of HipAngMean
  ThighAngL = 0;                   // Left thigh angle
  ThighAngL_T0InitValue = 0;       // Auxiliary parameter of T0 left thigh angle
  ThighAngR = 0;                   // Right thigh angle
  ThighAngR_T0InitValue = 0;       // Auxiliary parameter of T0 right thigh angle
  ThighAngMean = 0;                // (Left thigh angle + right thigh angle)/2
  ThighAngStd = 0;                 // Std(ThighAngMean) within certain time range
  HipAngDiffStd = 0;               // Std(HipAngDiff) within certain time range
  
  /* Initialize UID status flags */  
  mode = ExitState;   // Motion detection mode, default is Stop state
  PreMode = mode;
  tech = Stoop;       // BendingTech mode, default is stoop bending
  side = none;        // Asymmetric side, default is no asymmetric
  
  /* Initialize thresholds for specific subject */
  // Parameters for UID strategy
  UID_Subject1.ThrTrunkFleVel = 50;    // deg/s, Threshold for trunk flexion velocity
  UID_Subject1.ThrHipAngMean_1 = 40;   // deg, Threshold 1 for mean value of summation of left and right hip angle
  UID_Subject1.ThrHipAngMean_2 = 20;   // deg, Threshold 2 for mean value of summation of left and right hip angle  
  UID_Subject1.ThrHipAngDiff_1 = 10;   // deg, Threshold 1 for difference between left and right hip angle
  UID_Subject1.ThrHipAngStd_1 = 10;    // deg, Threshold 1 for standard deviation of mean hip angle
  UID_Subject1.ThrHipAngStd_2 = 5;     // deg, Threshold 2 for standard deviation of mean hip angle
  UID_Subject1.ThrHipAngStd_3 = 15;    // deg, Threshold 3 for standard deviation of mean hip angle
  UID_Subject1.ThrHipAngStd_4 = 10;    // deg, Threshold 3 for standard deviation of mean hip angle
  UID_Subject1.ThrHipVel = 10;         // deg/s, Threshold for mean hip angle velocity
  UID_Subject1.ThrTrunkFleAng_1 = 15;  // deg, Threshold 1 for trunk flexion angle
  UID_Subject1.ThrThighAngMean_2 = 15; // deg, Threshold 2 for mean thigh angle
  // Threshold 1 for mean thigh angle
  UID_Subject1.ThrThighAngMean_1 = UID_Subject1.ThrHipAngMean_1 - UID_Subject1.ThrTrunkFleAng_1;  
  // Threshold 2 for trunk flexion angle
  UID_Subject1.ThrTrunkFleAng_2 = UID_Subject1.ThrHipAngMean_1 - UID_Subject1.ThrThighAngMean_2;
  // Notice that ThrThighAngMean_2 should not be smaller than ThrThighAngMean_1 if ThighAng comes from
  // calculation of (HipAng - TrunkFleAng) instead of measurement
  if(UID_Subject1.ThrThighAngMean_2 < UID_Subject1.ThrThighAngMean_1) {
    UID_Subject1.ThrThighAngMean_2 = UID_Subject1.ThrThighAngMean_1;
    UID_Subject1.ThrTrunkFleAng_1 = UID_Subject1.ThrTrunkFleAng_2;
    // update again
    // Threshold 1 for mean thigh angle
    UID_Subject1.ThrThighAngMean_1 = UID_Subject1.ThrHipAngMean_1 - UID_Subject1.ThrTrunkFleAng_1;  
    // Threshold 2 for trunk flexion angle
    UID_Subject1.ThrTrunkFleAng_2 = UID_Subject1.ThrHipAngMean_1 - UID_Subject1.ThrThighAngMean_2;
  }
  UID_Subject1.ThrThighAngStd = 10;       // deg, Threshold for standard deviation of mean thigh angle
  UID_Subject1.ThrThighAngVel = 30;       // deg/s, Threshold for mean thigh angle velocity
  UID_Subject1.StdRange = 10;             // Standard deviation calculation range
  UID_Subject1.RatioTol = 0.2;            // Ratio tolerance related to hip angle for transition between standing and lowering&lifting
  // Following threshold setting are mainly for exit state
  UID_Subject1.ThrHipAngDiffStd = 20;     // deg, Threshold for standard deviation of difference between left and right hip angle
  UID_Subject1.ThrTrunkFleAngEMin = -15;  // deg, Threshold for allowable minimum trunk flexion angle
  UID_Subject1.ThrTrunkFleAngEMax = 140;  // deg, Threshold for allowable maximum trunk flexion angle
  UID_Subject1.ThrThighAngMeanEMin = -15; // deg, Threshold for allowable minimum thigh flexion angle 
  UID_Subject1.ThrThighAngMeanEMax = 140; // deg, Threshold for allowable maximum thigh flexion angle
  
  // Initialize auxiliary parameters for Std calculation and Finite state machine
  for(int i=0; i<FilterCycles; i++) {
    HipAngPreL[i] = 0;
    HipAngPreR[i] = 0;
    HipAngMeanPre[i] = 0;
    HipAngDiffPre[i] = 0;
    HipAngStdPre[i] = 0;
    HipAngDiffStdPre[i] = 0;
  }
  HipAngMeanBar = 0;
  HipAngDiffBar = 0;
}

/**
 * Pre-processing for sensor feedback related to high-level controller 
 * to make sure the initial status of sensor is good for calibration
 * @return int8_t - Sensor ready flag: 0-Not Ready; 1-Ready
 */
int8_t HLPreproSensorInit() {
  int8_t SensorReady;
  int8_t SensorReady_1;
  int8_t SensorReady_2;
  int8_t SensorReady_3;
  
  SensorReady = 0;
  // Initialize present yaw angle as 0 reference. Notice inside the function info will be collected
  // from IMUC simultaneously including: TrunkAng, TrunkVel
  yawAngleR20(ForcedInit,OperaitonAloIMUC);
  delay(1);
  // Collect info from ADC including: Potentiometets(HipAng), TorqueSensors/ForceSensors(Interaction force)
  // and Motor status 
  getADCaverage(1);
  delay(1);
  // Initialize the inital value for each sensor feedback
  // Notice to check the Initial value is ADC raw data or Processed data
  HipAngL_InitValue = Aver_ADC_value[PotentioLP1]/PotentioLP1_Sensitivity;
  HipAngR_InitValue = Aver_ADC_value[PotentioRP2]/PotentioRP2_Sensitivity;
  TrunkFleAng_InitValue = angleActualC[rollChan];
  // Here place program to check if these initial value of each sensor is near the expected position. 
  // If not, recalibration the initial value of the sensor feedback 
  if(HipAngL_InitValue > HipAngL_CaliValue + HipAngL_Tol || HipAngL_InitValue < HipAngL_CaliValue - HipAngL_Tol) 
  {SensorReady_1 = 0;}
  else {SensorReady_1 = 1;}
  if(HipAngR_InitValue > HipAngR_CaliValue + HipAngR_Tol || HipAngR_InitValue < HipAngR_CaliValue - HipAngR_Tol) 
  {SensorReady_2 = 0;}
  else {SensorReady_2 = 1;}
  if(TrunkFleAng_InitValue > TrunkFleAng_CaliValue + TrunkFleAng_Tol || TrunkFleAng_InitValue < TrunkFleAng_CaliValue - TrunkFleAng_Tol)
  {SensorReady_3 = 0;}
  else {SensorReady_3 = 1;}
  SensorReady = SensorReady_1*SensorReady_2*SensorReady_3;

  if(SensorReady == 0) {
    Serial.println("Sensor NotReady for High-level Controller.");
  }
  else {
    Serial.println("Sensor Ready for High-level Controller.");
  }
  return SensorReady;
  
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
    if(mode == Standing && (PreMode == ExitState || PreMode == Lifting || PreMode == Walking)) {
      if(YawAngleUpdate) {
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
        YawAngleUpdate = false;
      }
    }
    else {
      YawAngleUpdate = true;
    }
  }
  
}

/**
 * Processing sensor feedback for High-level closed-loop control and data sending
 */
void HLsensorFeedbackPro() {
//  // Smooth the hip angle feedback
//  MovingAverageFilter(PotentioLP1,5);
//  MovingAverageFilter(PotentioRP2,5);
//  // Smooth the trunk flexion angle feedback
//  MovingAverFilterIMUC(rollChan,5);

  /* Directly feedback from sensor */
  HipAngL = Aver_ADC_value[PotentioLP1]/PotentioLP1_Sensitivity - HipAngL_InitValue;
  HipAngR = Aver_ADC_value[PotentioRP2]/PotentioRP2_Sensitivity - HipAngR_InitValue;
  // when ESCON set 0~4V:-4000~4000rpm
  HipVelL_Motor = (Aver_ADC_value[MotorVeloL]-2)*4000*3;          //unit: deg/s 
  HipVelR_Motor = (Aver_ADC_value[MotorVeloR]-2)*4000*3;          //unit: deg/s 
  TrunkFleAng = angleActualC[rollChan] - TrunkFleAng_InitValue;
  TrunkYawAngPro();
  TrunkFleVel = velActualC[rollChan];
  /* Calculated from sensor feedback */
  HipAngMean = (HipAngL+HipAngR)/2;
  HipAngDiff = HipAngL-HipAngR;
  // ATTENTION that the historical hip angle are updated in HipAngStdCal(), threfore the hip velocity should be calculated after it !!!
  HipAngStd = HipAngStdCal(UID_Subject1.StdRange);   
  HipAngVelL = (HipAngPreL[FilterCycles-1] - HipAngPreL[FilterCycles-2])/looptime*1000;
  HipAngVelR = (HipAngPreR[FilterCycles-1] - HipAngPreR[FilterCycles-2])/looptime*1000;
  HipAngVel = (HipAngMeanPre[FilterCycles-1] - HipAngMeanPre[FilterCycles-2])/looptime*1000;
  ThighAngL = HipAngL - TrunkFleAng;
  ThighAngR = HipAngR - TrunkFleAng;
  ThighAngMean = (ThighAngL+ThighAngR)/2;
//  ThighAngStd = ThighAngStdCal(UID_Subject1.StdRange);
  HipAngDiffStd = HipAngDiffStdCal(UID_Subject1.StdRange);
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
 * Calculate standard deviation for HipAngMean within certain cycles
 * Here the population standard deviation is calculated
 * @param int - cycles: 1~FilterCycles
 * @return double - calculated standard deviation
 */
double HipAngStdCal(int cycles) {
  double interMean;
  double interValue;
  interValue = 0.0;
  if(cycles > FilterCycles) {
    cycles = FilterCycles;
  }
  else if(cycles < 1) {
    cycles = 1;
  }
  // get this times' mean value
  interMean = HipAngMeanBar + (HipAngMean - HipAngMeanPre[FilterCycles-cycles])/cycles;
  // update the data in the moving window
  for(int j=0; j<FilterCycles-1; j++) {
    HipAngPreL[j] = HipAngPreL[j+1];
    HipAngPreR[j] = HipAngPreR[j+1];
    HipAngMeanPre[j] = HipAngMeanPre[j+1];
    HipAngStdPre[j] = HipAngStdPre[j+1];
  }
  HipAngPreL[FilterCycles-1] = HipAngL;
  HipAngPreR[FilterCycles-1] = HipAngR;
  HipAngMeanPre[FilterCycles-1] = HipAngMean;
  // store the last time mean value
  HipAngMeanBar = interMean;
  // calculate standard deviation
  for(int j=FilterCycles-cycles; j<FilterCycles; j++) {
    interValue = interValue + pow(HipAngMeanPre[j]-interMean,2);
  }
  interValue = sqrt(interValue/cycles);
  HipAngStdPre[FilterCycles-1] = interValue;
  return interValue;  
}

/**
 * Calculate standard deviation for HipAngDiff within certain cycles
 * Here the population standard deviation is calculated
 * @param int - cycles: 1~FilterCycles
 * @return double - calculated standard deviation
 */
double HipAngDiffStdCal(int cycles) {
  double interMean;
  double interValue;
  interValue = 0.0;
  if(cycles > FilterCycles) {
    cycles = FilterCycles;
  }
  else if(cycles < 1) {
    cycles = 1;
  }
  // get this times' mean value
  interMean = HipAngDiffBar + (HipAngDiff - HipAngDiffPre[FilterCycles-cycles])/cycles;
  // update the data in the moving window
  for(int j=0; j<FilterCycles-1; j++) {
    HipAngDiffPre[j] = HipAngDiffPre[j+1];
    HipAngDiffStdPre[j] = HipAngDiffStdPre[j+1];
  }
  HipAngDiffPre[FilterCycles-1] = HipAngDiff;
  // store the last time mean value
  HipAngDiffBar = interMean;
  // calculate standard deviation
  for(int j=FilterCycles-cycles; j<FilterCycles; j++) {
    interValue = interValue + pow(HipAngDiffPre[j]-interMean,2);
  }
  interValue = sqrt(interValue/cycles);
  HipAngDiffStdPre[FilterCycles-1] = interValue;
  return interValue;    
}

/**
 * Conduct simple user intention detection and torque generation calculation as reference torque 
 * for low-level control based on sensor information feedback from force sensors, IMUs, Potentiometers
 * and Motor driver
 * @para unsigned char - control mode 1 for user intenntion detection: 1-xx algorithm, 2-xx algorithm
 *       unsigned char - control mode 2 for torque generation strategy: 1-xx strategy, 2-xx strategy
 */
void HLControl(uint8_t UIDMode, uint8_t RTGMode) {
  HL_UserIntentDetect(UIDMode);
  // Notice the enable/disable operation of Motor is within reference torque generation function
  HL_ReferTorqueGenerate(RTGMode);
}

/**
 * Conduct simple user intention detection 
 * @para unsigned char - control mode for user intenntion detection: 1-xx algorithm, 2-xx algorithm
 */
void HL_UserIntentDetect(uint8_t UIDMode) {
  // User intent detection strategy v1 (Referring to the 'Thoughts Keeping notbook')
  if(UIDMode == 1) {
    if(mode == Standing) {
      StandingPhase();
    }
    else if(mode == Walking) {
      WalkingPhase();
    }
    else if(mode == Lowering) {
      LoweringPhase();
    }
    else if(mode == Grasping) {
      GraspingPhase();
    }
    else if(mode == Lifting) {
      LiftingPhase();
    }
  }
  ExitPhase();
  // Reset the recorded state at To and peak moment
  if((mode == Standing && PreMode == Lifting)||mode == ExitState) {
    if(RecordStateReset) {
      HipAngL_T0InitValue = 0;
      HipAngR_T0InitValue = 0;
      TrunkFleAng_T0InitValue = 0;
      TrunkYaw_T0InitValue = 0;
      ThighAngL_T0InitValue = 0;
      ThighAngR_T0InitValue = 0;
      HipAngL_MaxValue = 0;
      HipAngR_MaxValue = 0;
      RecordStateReset = false;
    }
  }
  else {
    RecordStateReset = false;
  }
  
}

/**************** Functional function of each phase for the Finite-state machine ******************/
/**
 * System operation during Standing phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void StandingPhase() {
  if(abs(HipAngMean) < UID_Subject1.ThrHipAngMean_1 && HipAngStd > UID_Subject1.ThrHipAngStd_1) {
    if(ConThresReqCheck(UID_Subject1.ThrHipAngDiff_1,HipAngDiffPre,UID_Subject1.StdRange,1)) {
      mode = Walking;
      PreMode = Standing;
    }
  }
  else if(abs(HipAngMean) > UID_Subject1.ThrHipAngMean_1 && HipAngStd > UID_Subject1.ThrHipAngStd_3) {
    mode = Lowering;
    PreMode = Standing;
    // Record each angle when starting to lowering
    HipAngL_T0InitValue = HipAngL;
    HipAngR_T0InitValue = HipAngR;
    TrunkFleAng_T0InitValue = TrunkFleAng;
    TrunkYaw_T0InitValue = TrunkYawAng;
    ThighAngL_T0InitValue = ThighAngL;
    ThighAngR_T0InitValue = ThighAngR;
    // When lowering motion is detected, bending tech is classified in the meantime
    BendTechClassify();
  }
  else if (abs(HipAngMean) > UID_Subject1.ThrHipAngMean_1*(1+UID_Subject1.RatioTol)) {
    mode = Lowering;
    PreMode = Standing;
    // Record each angle when starting to lowering
    HipAngL_T0InitValue = HipAngL;
    HipAngR_T0InitValue = HipAngR;
    TrunkFleAng_T0InitValue = TrunkFleAng;
    TrunkYaw_T0InitValue = TrunkYawAng;
    ThighAngL_T0InitValue = ThighAngL;
    ThighAngR_T0InitValue = ThighAngR;
    // When lowering motion is detected, bending tech is classified in the meantime
    BendTechClassify();
  }
  else {mode = Standing;}
}

/**
 * System operation during Walking phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void WalkingPhase() {
  if(abs(HipAngMean) < UID_Subject1.ThrHipAngMean_2 && HipAngStd < UID_Subject1.ThrHipAngStd_2) {
    if(ConThresReqCheck(UID_Subject1.ThrHipAngDiff_1,HipAngDiffPre,UID_Subject1.StdRange,0)) {
      mode = Standing;
      PreMode = Walking;
    }
  }
  else {mode = Walking;}
}

/**
 * System operation during Lowering phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void LoweringPhase() {
  float Auxpara;
  Auxpara = PeakvalueDetect(HipAngMeanPre,1);
  if(Auxpara != -100 && HipAngStd < UID_Subject1.ThrHipAngStd_4) {
    mode = Grasping;
    PreMode = Lowering;
    //Record max hip joint bending angle
    if(HipAngL > HipAngL_MaxValue) {HipAngL_MaxValue = HipAngL;}
    if(HipAngR > HipAngR_MaxValue) {HipAngR_MaxValue = HipAngR;}
  }
  else {mode = Lowering;}
}

/**
 * System operation during Grasping phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void GraspingPhase() {
  float Auxpara;
  if(abs(HipAngMean) > UID_Subject1.ThrHipAngMean_1 && HipAngStd > UID_Subject1.ThrHipAngStd_4) {
    mode = Lifting;
    PreMode = Grasping;
  }
  else {
    mode = Grasping;
    Auxpara = PeakvalueDetect(HipAngMeanPre,1);
    if(Auxpara != -100) {
      //Keep updating max hip joint bending angle
      if(HipAngL > HipAngL_MaxValue) {HipAngL_MaxValue = HipAngL;}
      if(HipAngR > HipAngR_MaxValue) {HipAngR_MaxValue = HipAngR;}     
    }
  }
}

/**
 * System operation during Lifting phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void LiftingPhase() {
  if(abs(HipAngMean) < UID_Subject1.ThrHipAngMean_1 && HipAngStd < UID_Subject1.ThrHipAngStd_2) {
    mode = Standing;
    PreMode = Lifting;
  }
  else if (abs(HipAngMean) < UID_Subject1.ThrHipAngMean_1*(1-UID_Subject1.RatioTol)) {
    mode = Standing;
    PreMode = Lifting;
  }  
  else {mode = Lifting;}
}

/**
 * System operation during Exit phase: 
 *   Transmission condition detection 
 *   Reference torque generation strategy adjustment indication
 */
void ExitPhase() {
  if(TrunkFleAng < UID_Subject1.ThrTrunkFleAngEMin || TrunkFleAng > UID_Subject1.ThrTrunkFleAngEMax
  || ThighAngMean < UID_Subject1.ThrThighAngMeanEMin || ThighAngMean > UID_Subject1.ThrThighAngMeanEMax
  || HipAngDiffStd > UID_Subject1.ThrHipAngDiffStd) {
    mode = ExitState;
  }
  if(mode == ExitState) {
    if(abs(HipAngMean) < UID_Subject1.ThrHipAngMean_2 && HipAngStd < UID_Subject1.ThrHipAngStd_2) {
      if(ConThresReqCheck(UID_Subject1.ThrHipAngDiff_1,HipAngDiffPre,UID_Subject1.StdRange,0)) {
        mode = Standing;
        PreMode = ExitState;
      }
    }
  }
}

/**
 * Bending technique classification
 */
void BendTechClassify() {
  if(TrunkFleAng > UID_Subject1.ThrTrunkFleAng_1 && TrunkFleVel > UID_Subject1.ThrThighAngVel
  && ThighAngMean < UID_Subject1.ThrThighAngMean_1) {
    tech = Stoop;
  }
  else if(TrunkFleAng < UID_Subject1.ThrTrunkFleAng_2 && TrunkFleVel < UID_Subject1.ThrThighAngVel
  && ThighAngMean > UID_Subject1.ThrThighAngMean_2) {
    tech = Squat;
  }
  else {tech = SemiSquat;}
}

/**
 * Functional funciton to check continuously threshold requirement is meeted
 * @param float - threshold
 * @param float[] - The array to be checked
 * @param int - number of continous requirment satisfied items at once: 1~FilterCycles  
 * @param unsigned char - threshold requirements: 0- <Threshold, 1- >Threshold
 * @return bool - true: satisfied; false: not satisfied
 */
bool ConThresReqCheck(float threshold, float *Covalue, int cycles, uint8_t ThreRequire) {
  uint8_t Auxpara;
  uint8_t interPara;
  bool CheckFlag;
  Auxpara = 1;
  interPara = 1;
  if(cycles > FilterCycles) {
    cycles = FilterCycles;
  }
  else if(cycles < 1) {
    cycles = 1;
  }
  
  if(ThreRequire == 0) {
    for(int i=FilterCycles-cycles; i<FilterCycles; i++) {
      if(Covalue[i] < threshold) {interPara=1;}
      else {interPara=0;}
      Auxpara = Auxpara*interPara;
    }
  }
  else if(ThreRequire == 1) {
    for(int i=FilterCycles-cycles; i<FilterCycles; i++) {
      if(Covalue[i] > threshold) {interPara=1;}
      else {interPara=0;}
      Auxpara = Auxpara*interPara;
    }    
  }
  
  if(Auxpara == 1) {CheckFlag=true;}
  else {CheckFlag=false;}
  
  return CheckFlag;
}

/**
 * Functional funciton to detect peak angle
 * @param float[] - The array to be detected  
 * @param unsigned char - peak value mode: 0-detect minimum value, 1-detect maximum value
 * @return float - detected peakvalue
 */
float PeakvalueDetect(float *Pevalue, uint8_t Peakmode) {
  bool Findout;
  if(Peakmode == 0) {
    if(Pevalue[FilterCycles-2] < Pevalue[FilterCycles-3] && Pevalue[FilterCycles-2] < Pevalue[FilterCycles-1]) {Findout = true;}
    else {Findout = false;}
  }
  else if(Peakmode == 1) {
    if(Pevalue[FilterCycles-2] > Pevalue[FilterCycles-3] && Pevalue[FilterCycles-2] > Pevalue[FilterCycles-1]) {Findout = true;}
    else {Findout = false;}
  }

  if(Findout) {return Pevalue[FilterCycles-2];}
  else {return -100;}
}

/**
 * Functional function to detect the direction of angle variation
 * Notice this function usually be called after variation is exceed certain threshold
 * @param float[] - The array to be detected 
 * @param int - interval between process involved data point
 * @return unsigned char - detected direction: 1-increasing direction, 2-decreasing direction, 0-uncertain
 */
uint8_t DireStdDetect(float *Angvalue, int cycles) {
  uint8_t DirectFlag;
  float InTervalue;
  InTervalue = Angvalue[FilterCycles] - Angvalue[FilterCycles-cycles];
  if(InTervalue > 0) {
    DirectFlag = 1;
  }
  else if(InTervalue < 0) {
    DirectFlag = 2;
  }
  else {
    DirectFlag = 0;
  }
  return DirectFlag;
}
