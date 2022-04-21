/***********************************************************************
 * The Serial Communication definition and processing function
 **********************************************************************/

#include "SerialComu.h"

/*********************************** Serial communication definition ************************************/
bool upperControlFlag = true;        // mark if the upper computer is used for tethered operation or not 
char USART_RX_BUF[USART_REC_LEN];     // receiving buffer
int USART_RX_STA = 0;                 // recieving number flag
bool receiveCompleted = false;        // receiving completing flag
bool receiveContinuing = true;        // receiving continuous flag to avoid the multiple data format in buffer: xx\r\n+xx\r\n+xx...
bool SendPC_update = true;            // data sending to PC enable flag
char SwitchFlag = '0';                // mark if new command have recieved before sending data to PC
char USART_TX_BUF[USART_TX_LEN];      // sending buffer
int USART_TX_STA = 0;                 // sending number flag
// Protocol form: MxTLxxxxxALxxxxxVLxxxxxHLxxxxxTRxxxxxARxxxxxVRxxxxxHRxxxxxPxxxxxYxxxxxVxxxxxCLxxxxCRxxxxDLxxxxDRxxxxSxxx\r\n
bool SendItemFlag[16] = {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true};

/*********************************** Communication receiving data definition ************************************/
char inChar1;
char inChar2;


/**
 * @ PC to MCU Protocol: TLxxxxTRxxxxMxxx\r\n (0x0D,0x0A)
 * TLxxxx: Reference torque for left transmission system
 * TRxxxx: Reference torque for right transmission system
 * Mxxx: User motion mode indicator
 *   First number coresponds to MotionType
 *   Second number coresponds to AsymSide
 *   Third number corresponds to BendTech
 * Notice: With successful receiving process, USART_RX_STA indicates
 *         total reveived char number exclude terminator; and they are stored
 *         in USART_RX_BUF[0~USART_RX_STA-1] 
 *         Here only if terminaor is wrongly detected or data length exceed maximum 
 *         allowable length will lead to fail data receiving
 */
void receiveDatafromPC(void) {
  while(Serial.available() && receiveContinuing) {
    inChar1 = Serial.read();
    delayMicroseconds(20);  // delay to guarantee recieving correction under high buardrate
    // the string should end with \r
    if(inChar1 == '\r') {
      inChar2 = Serial.read();
      if(inChar2 == '\n') {
        receiveCompleted = true;       // correct receiving cycle
        receiveContinuing = false;     // finish this receiving cycle
      }
      else {
        USART_RX_STA = 0;
        receiveCompleted = false;    // incorrect receiving cycle
        receiveContinuing = false;   // finish this receiving cycle
      }      
    }
    else {
      if(USART_RX_STA < USART_REC_LEN) {
        USART_RX_BUF[USART_RX_STA] = inChar1;
        USART_RX_STA++;
      }
      // exceed the buffer size
      else {
        USART_RX_STA = 0;
        receiveCompleted = false;    // incorrect receiving cycle
        receiveContinuing = false;   // finish this receiving cycle
      }
    }
  }
  // Clear recieving buffer for next receiving cycle
  if(!receiveCompleted) {
    for(unsigned int i=0; i<strlen(USART_RX_BUF); i++) {
      USART_RX_BUF[i] = '\0';
    }
  }

}

/**
 * Split the data from PC to number
 * @ PC to MCU Protocol: TLxxxxTRxxxxMxxx\r\n (0x0D,0x0A)
 */
void receivedDataPro(void) {
  /* Remind that the recieved data are stored in receiving buffer USART_RX_BUF[0~USART_RX_STA-1]: 
     TLxxxxTRxxxxMxxx if the receiving cycle is correctly completed */
  int tempMode;
  int tempSide;
  int tempTech;
  // Length and first character are checked to esure the command is recieved correctly
  if(receiveCompleted && USART_RX_STA == RevievCharNum) {
    // Check if the character is correct
    if(USART_RX_BUF[0] == 'T' && USART_RX_BUF[1] == 'L') {
      // Calculate reference torque for left system
      desiredTorqueL = (USART_RX_BUF[2]-48)*10+(USART_RX_BUF[3]-48)*1+(USART_RX_BUF[4]-48)*0.1+(USART_RX_BUF[5]-48)*0.01;
      // Check and limit the reasonable torque range 0~LimitInput
      if(desiredTorqueL >= LimitInput) {
        desiredTorqueL = LimitInput;
      }
      else if(desiredTorqueL < 0) {
        desiredTorqueL = 0;
      } 
    }
    if(USART_RX_BUF[6] == 'T' && USART_RX_BUF[7] == 'R') {     
      // Calculate reference torque for right system
      desiredTorqueR = (USART_RX_BUF[8]-48)*10+(USART_RX_BUF[9]-48)*1+(USART_RX_BUF[10]-48)*0.1+(USART_RX_BUF[11]-48)*0.01;
      // Check and limit the reasonable torque range 0~LimitInput
      if(desiredTorqueR >= LimitInput) {
        desiredTorqueR = LimitInput;
      }
      else if(desiredTorqueR < 0) {
        desiredTorqueR = 0;
      } 
    }
    if(USART_RX_BUF[12] == 'M') {
      // MotionType
      PreMode = mode;
      tempMode = USART_RX_BUF[13]-48;
      if(tempMode == 1) {mode = ExitState;}
      else if(tempMode == 2) {mode = Standing;}
      else if(tempMode == 3) {mode = Walking;}
      else if(tempMode == 4) {mode = Lowering;}
      else if(tempMode == 5) {mode = Grasping;}
      else if(tempMode == 6) {mode = Lifting;}
      else {mode = StopState;}
      // AsymSide
      tempSide = USART_RX_BUF[14]-48;
      if(tempSide == 1) {side = left;}
      else if(tempSide == 2) {side = right;}
      else {side = none;}
      // BendTech
      tempTech = USART_RX_BUF[15]-48;
      if(tempTech == 0) {tech = Stoop;}
      else if(tempTech == 2) {tech = SemiSquat;}
      else {tech = Squat;}      
    }

     /* ONLY FOR MOTOR TESTING */
//    // Enable motor
//    if(side == left || side == right) {
//      // Enable left and right
//      digitalWrite(MotorEnableL,HIGH);   
//      digitalWrite(MotorEnableR,HIGH);     
//    }
//    else if(side == none) {
//      // Disable left and right 
//      digitalWrite(MotorEnableL,LOW);     
//      digitalWrite(MotorEnableR,LOW);     
//    } 
  }
  
  // Mark unsuccessful command receive from PC
  else if(USART_RX_STA != RevievCharNum) {
    receiveCompleted = false;
    // Clear recieving buffer for next receiving cycle
    for(unsigned int i=0; i<strlen(USART_RX_BUF); i++) {
      USART_RX_BUF[i] = '\0';
    }    
  }

}

/**
 * @ MCU to PC protocol: MxTLxxxxxALxxxxxVLxxxxxHLxxxxxTRxxxxxARxxxxxVRxxxxxHRxxxxxPxxxxxYxxxxxVxxxxxCLxxxxCRxxxxDLxxxxDRxxxxSxxx\r\n
 * TL/Rxxxxx: (Nm) Torque feedback for left/right transmission system 
 *                 first number indicate sign: 0 for -, 1 for +
 * AL/Rxxxxx: (deg) Hip angle feedback from potentiometer
 *                  first number indicate sign: 0 for -, 1 for +
 * VL/Rxxxxx: (deg/s) Hip angular velocity feedback
 *                    first number indicate sign: 0 for -, 1 for +
 * HL/Rxxxxx: (deg/s^2) Hip angular acceleration feedback
 *                    first number indicate sign: 0 for -, 1 for +  
 * Pxxxxx: (deg) Pitch angle for trunk
 *               first number indicate sign: 0 for -, 1 for +
 * Yxxxxx: (deg) yaw angle for trunk
 *               first number indicate sign: 0 for -, 1 for + 
 * Vxxxxx: (deg/s) Pitch angular velocity for trunk
 *                 first number indicate sign: 0 for -, 1 for +
 * CL/Rxxxx: PWM Cycle Duty
 *           first number indicate sign: 0 for -(loosening cable), 1 for +(tighting cable)
 * DL/Rxxxx: Desired torque command
 * Sxxx    : User motion status indicator
 *   First number coresponds to MotionType
 *   Second number coresponds to AsymSide
 *   Third number corresponds to BendTech 
 * Mx: Marking flag to show if the MCU data is real-time with successful receiving last command from PC
 * Notice: The last two are terminator for PC receiveing '\r\n'
 */
void sendDatatoPC() {
  int dec;
  int position;
  int8_t SignMark;
  unsigned char inter;
  position = 0;

  // Check if the command is recieved
  if(receiveCompleted) {
    if(SwitchFlag == '0') {
      SwitchFlag = '1';
    }
    else {
      SwitchFlag = '0';
    }
  }
  // Mx
  USART_TX_BUF[position++] = 'M';
  USART_TX_BUF[position++] = SwitchFlag;
  // TLxxxxx
  if(SendItemFlag[0] == true) {
    USART_TX_BUF[position++] = 'T';
    USART_TX_BUF[position++] = 'L';
    SignMark = Value_sign(Estimated_TdL);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xx.xx
    dec = SignMark*Estimated_TdL*Calcu_Pow(10,2);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }
  }
  // ALxxxxx
  if(SendItemFlag[1] == true) {
    USART_TX_BUF[position++] = 'A';
    USART_TX_BUF[position++] = 'L';
    SignMark = Value_sign(HipAngL);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xxx.x
    dec = SignMark*HipAngL*Calcu_Pow(10,1);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }      
  }
  // VLxxxxx
  if(SendItemFlag[2] == true) {
    USART_TX_BUF[position++] = 'V';
    USART_TX_BUF[position++] = 'L';
    SignMark = Value_sign(HipAngVelL);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xxx.x
    dec = SignMark*HipAngVelL*Calcu_Pow(10,1);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }      
  }
  // HLxxxxx
  if(SendItemFlag[3] == true) {
    USART_TX_BUF[position++] = 'H';
    USART_TX_BUF[position++] = 'L';
    SignMark = Value_sign(HipAngAccL);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xxx.x
    dec = SignMark*HipAngAccL*Calcu_Pow(10,1);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }      
  }  
  // TRxxxxx
  if(SendItemFlag[4] == true) {
    USART_TX_BUF[position++] = 'T';
    USART_TX_BUF[position++] = 'R';
    SignMark = Value_sign(Estimated_TdR);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xx.xx
    dec = SignMark*Estimated_TdR*Calcu_Pow(10,2);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }
  }
  // ARxxxxx
  if(SendItemFlag[5] == true) {
    USART_TX_BUF[position++] = 'A';
    USART_TX_BUF[position++] = 'R';
    SignMark = Value_sign(HipAngR);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xxx.x
    dec = SignMark*HipAngR*Calcu_Pow(10,1);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }      
  }
  // VRxxxxx
  if(SendItemFlag[6] == true) {
    USART_TX_BUF[position++] = 'V';
    USART_TX_BUF[position++] = 'R';
    SignMark = Value_sign(HipAngVelR);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xxx.x
    dec = SignMark*HipAngVelR*Calcu_Pow(10,1);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }      
  }  
  // HRxxxxx
  if(SendItemFlag[7] == true) {
    USART_TX_BUF[position++] = 'H';
    USART_TX_BUF[position++] = 'R';
    SignMark = Value_sign(HipAngAccR);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xxx.x
    dec = SignMark*HipAngAccR*Calcu_Pow(10,1);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }      
  }
  // Pxxxxx, Notice the X-axis (practical pitch angle) is assigned as roll channel in the program
  if(SendItemFlag[8] == true) {
    USART_TX_BUF[position++] = 'P';
    SignMark = Value_sign(TrunkFleAng);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xxx.x
    dec = SignMark*TrunkFleAng*Calcu_Pow(10,1);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }      
  }
  // Yxxxxx
  if(SendItemFlag[9] == true) {
    USART_TX_BUF[position++] = 'Y';
    SignMark = Value_sign(TrunkYawAng);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xxx.x
    dec = SignMark*TrunkYawAng*Calcu_Pow(10,1);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }      
  }
  // Vxxxxx
  if(SendItemFlag[10] == true) {
    USART_TX_BUF[position++] = 'V';
    SignMark = Value_sign(TrunkFleVel);
    if(SignMark == PosSign) {
      USART_TX_BUF[position++] = PosSign+48;
    }
    else {
      USART_TX_BUF[position++] = NegSign+48;
    }
    // ±xxx.x
    dec = SignMark*TrunkFleVel*Calcu_Pow(10,1);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }             
  }
  // CLxxxx
  if(SendItemFlag[11] == true) {
    USART_TX_BUF[position++] = 'C';
    USART_TX_BUF[position++] = 'L';
    // ±xxx
    USART_TX_BUF[position++] = PWMSignL+48;
    dec = PWM_commandL;
    for(int t=0; t<3; t++) {
      inter = (dec/Calcu_Pow(10,2-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }
  }
  // CRxxxx
  if(SendItemFlag[12] == true) {
    USART_TX_BUF[position++] = 'C';
    USART_TX_BUF[position++] = 'R';     
    // ±xxx
    USART_TX_BUF[position++] = PWMSignR+48;
    dec = PWM_commandR;
    for(int t=0; t<3; t++) {
      inter = (dec/Calcu_Pow(10,2-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }
  }
  // DLxxxx
  if(SendItemFlag[13] == true) {
    USART_TX_BUF[position++] = 'D';
    USART_TX_BUF[position++] = 'L';     
    // xx.xx
    dec = desiredTorqueL*Calcu_Pow(10,2);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }
  }  
  // DRxxxx
  if(SendItemFlag[14] == true) {
    USART_TX_BUF[position++] = 'D';
    USART_TX_BUF[position++] = 'R';     
    // xx.xx
    dec = desiredTorqueR*Calcu_Pow(10,2);
    for(int t=0; t<4; t++) {
      inter = (dec/Calcu_Pow(10,3-t))%10;
      USART_TX_BUF[position++] = inter+48;
    }
  } 
  // Sxxx
  if(SendItemFlag[15] == true) {
    USART_TX_BUF[position++] = 'S';
    //xxx
    // MotionType
    if(mode == ExitState) {inter = 1;}
    else if(mode == Standing) {inter = 2;}
    else if(mode == Walking) {inter = 3;}
    else if(mode == Lowering) {inter = 4;}
    else if(mode == Grasping) {inter = 5;}
    else if(mode == Lifting) {inter = 6;}
    USART_TX_BUF[position++] = inter+48;
    // AsymSide
    if(side == none) {inter = 0;}
    else if(side == left) {inter = 1;}
    else if(side == right) {inter = 2;}
    USART_TX_BUF[position++] = inter+48;
    // BendTech
    if(tech == Stoop) {inter = 0;}
    else if(tech == Squat) {inter = 1;}
    else if(tech == SemiSquat){inter = 2;}    
    USART_TX_BUF[position++] = inter+48;     
  }
  Serial.println(USART_TX_BUF); // Speed up frequency
}

/**
 * calculate m^n
 * @param signed char - based m
 * @param signed char - index n
 * @return signed int - the results of m^n
 */
int32_t Calcu_Pow(int8_t m, int8_t n) {
  int32_t result = 1;  
  while(n--) result*=m;    
  return result;  
}

/**
 * @param double - data to judge for sign
 * @return signed char - the sign of the value
 */
int8_t Value_sign(double data)
{
  if(data >= 0)
    return 1;
  else
    return -1;
}
