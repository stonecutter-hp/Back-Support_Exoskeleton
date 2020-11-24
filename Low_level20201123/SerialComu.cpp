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
char USART_TX_BUF[USART_TX_LEN];      // sending buffer
int USART_TX_STA = 0;                 // sending number flag

/*********************************** Communication receiving data definition ************************************/
float desiredTorqueL;    // desired motor torque of left motor
float desiredTorqueR;    // desired motor torque of right motor
int inChar;


/**
 * @ PC to MCU Protocol: TLxxxxxTRxxxxxMxx\r (0x0D)
 * TLxxxxx: Reference torque for left transmission system, 
 *          first number indicate sign: 0 for -, 1 for + 
 * TRxxxxx: Reference torque for right transmission system,
 *          first number indicate sign: 0 for -, 1 for + 
 * Mxx: Detected user motion mode
 * Notice: With successful receiving process, USART_RX_STA indicates
 *         total reveived char number exclude '\r'; and they are stored
 *         in USART_RX_BUF[0~USART_RX_STA-1], i.e., TLxxxxxTRxxxxxMxx 
 */
void receiveDatafromPC(void) {
  while(Serial.available() && receiveContinuing) {
    inChar = Serial.read();
    delayMicroseconds(20);  // delay to guarantee recieving correction under high buardrate
    // the string should end with \r
    if(inChar == '\r') {
      receiveCompleted = true;       // correct receiving cycle
      receiveContinuing = false;     //finish this receiving cycle
    }
    else {
      if(USART_RX_STA < USART_REC_LEN) {
        USART_RX_BUF[USART_RX_STA] = inChar;
        USART_RX_STA++;
      }
      // exceed the buffer size
      else {
        USART_RX_STA = 0;
        receiveCompleted = false;    // incorrect receiving cycle
        receiveContinuing = false;   //finish this receiving cycle
      }
    }
  }

}

/**
 * Split the data from PC to number
 * PC to MCU Protocol: TLxxxxxTRxxxxxMxx\r (0x0D)
 */
void receivedDataPro(void) {
  // Remind that the recieved data are stored in receiving buffer USART_RX_BUF[0~USART_RX_STA-1]: TLxxxxTRxxxxMxx
  // if the receiving cycle is correct completed

  // Length and first character are checked to esure the command is recieved correctly
  if(receiveCompleted && USART_RX_STA == 17) {
    // Check if the character is correct
    if(USART_RX_BUF[0] == 'T' && USART_RX_BUF[1] == 'L') {
      // Calculate reference torque for left system
      desiredTorqueL = (USART_RX_BUF[3]-48)*10+(USART_RX_BUF[4]-48)*1+(USART_RX_BUF[5]-48)*0.1+(USART_RX_BUF[6]-48)*0.01;
      // Check and limit the reasonable torque range 0~LimitInput
      if(desiredTorqueL >= LimitInput) {
        desiredTorqueL = LimitInput;
      }
      else if(desiredTorqueL < 0) {
      	desiredTorqueL = 0;
      } 
    }

    // Here add more process for data decomposition ...

  }

}

/**
 * @ MCU to PC protocol: ALxxxxLLxxxxTLxxxxARxxxxLRxxxxTRxxxxPxxxxYxxxxx\r\n
 * AL/Rxxxx: Support beam angle for left/right transmission system 
 * LL/Rxxxx: Load cell for cable force of left/right transmission system
 * TL/Rxxxx: Potentiometer/IMU feedback for angle between left/right thigh and trunk
 * Pxxxx: Pitch angle for trunk
 * Yxxxxx: Yaw angle for trunk
 *         first number indicate sign: 0 for -, 1 for + 
 * Notice: The last two is end character for PC receiveing '\r\n'
 */
void sendDatatoPC(void) {
  // Here code Sending data according to test/operation requirement
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
