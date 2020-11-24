/***********************************************************************
 * The Serial Communication definition and processing function
 **********************************************************************/

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "IIC.h"
#include "Arduino.h"
#include "Control.h"

/*********************************** Serial communication definition ************************************/
#define USART_REC_LEN 200             // define the maximum number of received bytes
#define USART_TX_LEN 200              // define the maximum number of sending bytes
extern char USART_RX_BUF[USART_REC_LEN];     // receiving buffer
extern int USART_RX_STA;                 // recieving number flag
extern bool receiveCompleted;        // receiving completing flag
extern bool receiveContinuing;       // receiving continuous flag to avoid the multiple data format in buffer: xx\r\n+xx\r\n+xx...
extern bool SendPC_update;            // data sending to PC enable flag
extern char USART_TX_BUF[USART_TX_LEN];      // sending buffer
extern int USART_TX_STA;                 // sending number flag

/*********************************** Communication receiving data definition ************************************/
extern float desiredTorqueL;    // desired motor torque of left motor
extern float desiredTorqueR;    // desired motor torque of right motor
extern int inChar;



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
void receiveDatafromPC(void);

/**
 * Split the data from PC to number
 * @ PC to MCU Protocol: TLxxxxxTRxxxxxMxx\r (0x0D)
 */
void receivedDataPro(void);

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
void sendDatatoPC(void);

/**
 * calculate m^n
 * @param signed char - based m
 * @param signed char - index n
 * @return signed int - the results of m^n
 */
int32_t Calcu_Pow(int8_t m, int8_t n);

/**
 * @param double - data to judge for sign
 * @return signed char - the sign of the value
 */
int8_t Value_sign(double data);








#endif
