/***********************************************************************
 * The Serial Communication definition and processing function
 **********************************************************************/

#include "SerialComu.h"

/*********************************** Serial communication definition ************************************/
char USART_RX_BUF[USART_REC_LEN];     // receiving buffer
int USART_RX_STA = 0;                 // recieving number flag
bool receiveCompleted = false;        // receiving completing flag
bool receiveContinuing = false;       // receiving continuous flag to avoid the multiple data format in buffer: xx\r\n+xx\r\n+xx...
bool SendPC_update = true;            // data sending to PC enable flag
char SwitchFlag = '0';                // mark if new command have recieved before sending data to PC
char USART_TX_BUF[USART_TX_LEN];      // sending buffer
int USART_TX_STA = 0;                 // sending number flag
// TLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxx\r\n
bool SendItemFlag[9] = {true, true, true, true, true, true, true, true, true};

/*********************************** Communication receiving data definition ************************************/
float desiredTorqueL;    // desired motor torque of left motor
float desiredTorqueR;    // desired motor torque of right motor
// motion type: 1-other motion;       2-Symmetric Holding; 
//              3-Asymmetric Holding; 4-Asymmetric Lowering;
//              5-Asymmetric Lifting; 6-Symmetric Lowering;
//              7-Symmetric Lifting;  0-Stop state
uint8_t mode;            // detected motion mode
uint8_t PreMode;         // last time's motion mode
// 1-left; 2-right; 0-none
uint8_t side;            // Asymmetric side
char inChar1;
char inChar2;


/**
 * @ PC to MCU Protocol: TLxxxxTRxxxxMxx\r\n (0x0D,0x0A)
 * TLxxxx: Reference torque for left transmission system
 * TRxxxx: Reference torque for right transmission system
 * Mxx: Detected user motion mode
 * Notice: With successful receiving process, USART_RX_STA indicates
 *         total reveived char number exclude '\r\n'; and they are stored
 *         in USART_RX_BUF[0~USART_RX_STA-1], i.e., TLxxxxTRxxxxMxx 
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
 * PC to MCU Protocol: TLxxxxTRxxxxMxx\r\n (0x0D,0x0A)
 */
void receivedDataPro(void) {
  // Remind that the recieved data are stored in receiving buffer USART_RX_BUF[0~USART_RX_STA-1]: TLxxxxTRxxxxMxx
  // if the receiving cycle is correct completed

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
      // Calculate reference torque for left system
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
 * @ MCU to PC protocol: MxTLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxx\r\n
 * TL/Rxxxx: (Nm) Torsion spring torque for left/right transmission system 
 * LL/Rxxxx: (N) Load cell for cable force of left/right transmission system
 * AL/Rxxxxx: (deg) Potentiometer/IMU feedback for angle between left/right thigh and vertical direction
 *                  first number indicate sign: 0 for -, 1 for +
 * Pxxxxx: (deg) Pitch angle for trunk
 * Yxxxxx: (deg) yaw angle for trunk
 *               first number indicate sign: 0 for -, 1 for + 
 * Vxxxxx: (deg/s) Pitch angular velocity for trunk
 *                 first number indicate sign: 0 for -, 1 for +
 * Mx: Marking flag to show if the MCU data is real-time with successful receiving last command from PC
 * Notice: The last two is end character for PC receiveing '\r\n'
 */
void sendDatatoPC(void) {
  int dec;
  int position;
  int8_t SignMark;
  unsigned char inter;
  position = 0;
  if(SendPC_update == true) {
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
  	  dec = Estimated_PoAssistiveTorqueL*Calcu_Pow(10,2);
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
      dec = Aver_ADC_value[LoadCellL]*Calcu_Pow(10,1);
  	  for(int t=0; t<4; t++) {
  	  	inter = (dec/Calcu_Pow(10,3-t))%10;
  	  	USART_TX_BUF[position++] = inter+48;
  	  }      
  	}
  	// ALxxxxx
  	if(SendItemFlag[2] == true) {
      USART_TX_BUF[position++] = 'A';
      USART_TX_BUF[position++] = 'L';
      SignMark = Value_sign(angleActualA[pitchChan]);
      if(SignMark == PosSign) {
      	USART_TX_BUF[position++] = PosSign+48;
      }
      else {
        USART_TX_BUF[position++] = NegSign+48;
      }
      // ±xx.xx
      dec = SignMark*angleActualA[pitchChan]*Calcu_Pow(10,2);
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
  	  dec = Estimated_PoAssistiveTorqueR*Calcu_Pow(10,2);
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
      dec = Aver_ADC_value[LoadCellR]*Calcu_Pow(10,1);
  	  for(int t=0; t<4; t++) {
  	  	inter = (dec/Calcu_Pow(10,3-t))%10;
  	  	USART_TX_BUF[position++] = inter+48;
  	  }      
  	}
  	// ALxxxxx
  	if(SendItemFlag[5] == true) {
      USART_TX_BUF[position++] = 'A';
      USART_TX_BUF[position++] = 'R';
      SignMark = Value_sign(angleActualB[pitchChan]);
      if(SignMark == PosSign) {
      	USART_TX_BUF[position++] = PosSign+48;
      }
      else {
        USART_TX_BUF[position++] = NegSign+48;
      }
      // ±xx.xx
      dec = SignMark*angleActualB[pitchChan]*Calcu_Pow(10,2);
  	  for(int t=0; t<4; t++) {
  	  	inter = (dec/Calcu_Pow(10,3-t))%10;
  	  	USART_TX_BUF[position++] = inter+48;
  	  }      
  	}
  	// Pxxxxx, Notice the X-axis (practical pitch angle) is assigned as roll channel in the program
  	if(SendItemFlag[6] == true) {
      USART_TX_BUF[position++] = 'P';
      SignMark = Value_sign(angleActualC[rollChan]);
      if(SignMark == PosSign) {
      	USART_TX_BUF[position++] = PosSign+48;
      }
      else {
        USART_TX_BUF[position++] = NegSign+48;
      }
      // ±xxx.x
      dec = SignMark*angleActualC[rollChan]*Calcu_Pow(10,1);
  	  for(int t=0; t<4; t++) {
  	  	inter = (dec/Calcu_Pow(10,3-t))%10;
  	  	USART_TX_BUF[position++] = inter+48;
  	  }      
  	}
  	// Yxxxxx
  	if(SendItemFlag[7] == true) {
      USART_TX_BUF[position++] = 'Y';
      SignMark = Value_sign(angleActualC[yawChan]);
      if(SignMark == PosSign) {
      	USART_TX_BUF[position++] = PosSign+48;
      }
      else {
        USART_TX_BUF[position++] = NegSign+48;
      }
      // ±xx.xx
      dec = SignMark*angleActualC[yawChan]*Calcu_Pow(10,2);
  	  for(int t=0; t<4; t++) {
  	  	inter = (dec/Calcu_Pow(10,3-t))%10;
  	  	USART_TX_BUF[position++] = inter+48;
  	  }      
  	}
  	// Vxxxxx
  	if(SendItemFlag[8] == true) {
  	  USART_TX_BUF[position++] = 'V';
  	  SignMark = Value_sign(TrunkFlexionVel);
      if(SignMark == PosSign) {
      	USART_TX_BUF[position++] = PosSign+48;
      }
      else {
        USART_TX_BUF[position++] = NegSign+48;
      }
      // ±xxx.x
      dec = SignMark*TrunkFlexionVel*Calcu_Pow(10,1);
  	  for(int t=0; t<4; t++) {
  	  	inter = (dec/Calcu_Pow(10,3-t))%10;
  	  	USART_TX_BUF[position++] = inter+48;
  	  }         	  
  	}
    // Serial.print(USART_TX_BUF);
    // Serial.flush();
    // Serial.print('\r');
    // Serial.flush();
    // Serial.print('\n');
    // Serial.flush();	
    Serial.println(USART_TX_BUF); // Speed up frequency
  }
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
