/***********************************************************************
 * The Serial Communication definition and processing function
 **********************************************************************/

#include "SerialComu.h"

/*********************************** Serial communication definition ************************************/
char USART_RX_BUF[USART_REC_LEN];     // receiving buffer
int USART_RX_STA = 0;                 // recieving number flag
bool receiveCompleted = false;        // receiving completing flag
bool receiveContinuing = true;        // receiving continuous flag to avoid the multiple data format in buffer: xx\r\n+xx\r\n+xx...
bool SendPC_update = true;            // data sending to PC enable flag
char SwitchFlag = '0';                // mark if new command have recieved before sending data to PC
char USART_TX_BUF[USART_TX_LEN];      // sending buffer
int USART_TX_STA = 0;                 // sending number flag
// MxTLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxxCLxxxxCRxxxx\r\n
bool SendItemFlag[11] = {true, true, true, true, true, true, true, true, true, true, true};

/*********************************** Communication receiving data definition ************************************/
float desiredTorqueL;    // desired motor torque of left motor
float desiredTorqueR;    // desired motor torque of right motor
float PredesiredTorqueL; // previous desired assistive torque of left torque transmission system
float PredesiredTorqueR; // previous desired assistive torque of right torque transmission system
uint8_t mode;            // detected motion mode
uint8_t PreMode;         // last time's motion mode
uint8_t side;            // another auxiliary indicator for asymmetric and low-level compensation term from high-level control


/**
 * @ PC to MCU Protocol: TLxxxxTRxxxxMxx\r\n (0x0D,0x0A)
 * TLxxxx: Reference torque for left transmission system
 * TRxxxx: Reference torque for right transmission system
 * Mxx: Detected user motion mode and low-level control mode indicator
 *  First number indicates motion type: 0-Stop state;      1-Exit state; 
 *                                      2-Standing;        3-Walking;
 *                                      4-Lowering;        5-Grasping;
 *                                      6-Lifting;         7-Holding;
 *  Second number indicates asymmetric side & low-level control compensation model:
 *                                      0-none; 1-left; 2-right; 
 *                                      3 (0+3)-none + fricCom;
 *                                      4 (1+3)-left + fricCom;
 *                                      5( 2+3)-right + fricCom
 * Notice: With successful receiving process, USART_RX_STA indicates
 *         total reveived char number exclude '\r\n'; and they are stored
 *         in USART_RX_BUF[0~USART_RX_STA-1], i.e., TLxxxxTRxxxxMxx 
 *         Here only if terminaor is wrongly detected or data length exceed maximum 
 *         allowable length will lead to fail data receiving
 */
void receiveDatafromPC(void) {
  char inChar1;
  char inChar2;
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
 * PC to MCU Protocol: TLxxxxTRxxxxMxx\r\n (0x0D,0x0A)
 */
void receivedDataPro(void) {
  /* Remind that the recieved data are stored in receiving buffer USART_RX_BUF[0~USART_RX_STA-1]: 
     TLxxxxTRxxxxMxx if the receiving cycle is correctly completed */

  // Length and first character are checked to esure the command is recieved correctly
  if(receiveCompleted && USART_RX_STA == RevievCharNum) {
    // Check if the character is correct
    if(USART_RX_BUF[0] == 'T' && USART_RX_BUF[1] == 'L') {
      // Update previous reference torque
      PredesiredTorqueL = desiredTorqueL;
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
      // Update previous reference torque
      PredesiredTorqueR = desiredTorqueR;      
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
      PreMode = mode;
      mode = USART_RX_BUF[13]-48;
      side = USART_RX_BUF[14]-48;
    }    
    // Here add more process for data decomposition ...
    
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
 * @ MCU to PC protocol: MxTLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxxCLxxxxCRxxxx\r\n
 * TL/Rxxxx: (Nm) Torque feedback for left/right transmission system 
 * LL/Rxxxx: (N) Load cell feedback for cable force of left/right transmission system
 * AL/Rxxxxx: (deg) Potentiometer feedback for hip angle feedback
 *                  first number indicate sign: 0 for -, 1 for +
 * Pxxxxx: (deg) Pitch angle for trunk
 *               first number indicate sign: 0 for -, 1 for +
 * Yxxxxx: (deg) yaw angle for trunk
 *               first number indicate sign: 0 for -, 1 for + 
 * Vxxxxx: (deg/s) Pitch angular velocity for trunk
 *                 first number indicate sign: 0 for -, 1 for +
 * CL/Rxxxx: PWM Cycle Duty
 *           first number indicate sign: 0 for -(loosening cable), 1 for +(tighting cable)
 * Mx: Marking flag to show if the MCU data is real-time with successful receiving last command from PC
 * Notice: The last two are terminator for PC receiveing '\r\n'
 */
void sendDatatoPC(void) {
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
	// TLxxxx
	if(SendItemFlag[0] == true) {
	  USART_TX_BUF[position++] = 'T';
	  USART_TX_BUF[position++] = 'L';
    // xx.xx
	  dec = Feedback_TdL*Calcu_Pow(10,2);
	  for(int t=0; t<4; t++) {
	  	inter = (dec/Calcu_Pow(10,3-t))%10;
	  	USART_TX_BUF[position++] = inter+48;
	  }
	}
	// LLxxxx
	if(SendItemFlag[1] == true) {
    USART_TX_BUF[position++] = 'L';
    USART_TX_BUF[position++] = 'L';
    // xxx.x
    dec = Estimated_FcL*Calcu_Pow(10,1);
	  for(int t=0; t<4; t++) {
	  	inter = (dec/Calcu_Pow(10,3-t))%10;
	  	USART_TX_BUF[position++] = inter+48;
	  }      
	}
	// ALxxxxx
	if(SendItemFlag[2] == true) {
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
	// TRxxxx
	if(SendItemFlag[3] == true) {
	  USART_TX_BUF[position++] = 'T';
	  USART_TX_BUF[position++] = 'R';
    // xx.xx
	  dec = Feedback_TdR*Calcu_Pow(10,2);
	  for(int t=0; t<4; t++) {
	  	inter = (dec/Calcu_Pow(10,3-t))%10;
	  	USART_TX_BUF[position++] = inter+48;
	  }
	}
	// LRxxxx
	if(SendItemFlag[4] == true) {
    USART_TX_BUF[position++] = 'L';
    USART_TX_BUF[position++] = 'R';
    // xxx.x
    dec = Estimated_FcR*Calcu_Pow(10,1);
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
	// Pxxxxx, Notice the X-axis (practical pitch angle) is assigned as roll channel in the program
	if(SendItemFlag[6] == true) {
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
	if(SendItemFlag[7] == true) {
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
	if(SendItemFlag[8] == true) {
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
  if(SendItemFlag[9] == true) {
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
  if(SendItemFlag[10] == true) {
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
