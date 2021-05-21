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
// Protocol like form: TLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxx\r\n
bool SendItemFlag[9] = {true, true, true, true, true, true, true, true, true};

/*********************************** Communication receiving data definition ************************************/
char inChar1;
char inChar2;


/**
 * @ PC to MCU Protocol: -----
 * Notice: With successful receiving process, USART_RX_STA indicates
 *         total reveived char number exclude terminator; and they are stored
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
 * @ PC to MCU Protocol: -----
 */
void receivedDataPro(void) {
  // Remind that the recieved data are stored in receiving buffer USART_RX_BUF[0~USART_RX_STA-1] 
  // exclued terminator such as: TLxxxxTRxxxxMxx if the receiving cycle is correct completed

  // Length and first character are checked to esure the command is recieved correctly
  if(receiveCompleted && USART_RX_STA == RevievCharNum) {   
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
 * @ MCU to PC protocol: -----
 * Notice: The last two is end character for PC receiveing '\r\n'
 */
void sendDatatoPC(void) {
  Serial.println(mode);
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
