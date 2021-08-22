/***********************************************************************
 * The Serial Communication definition and processing function
 **********************************************************************/

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "IIC.h"
#include "Arduino.h"
#include "Control.h"
#include "RefTG.h"
#include "IMU.h"

/*********************************** Serial communication definition ************************************/
#define USART_REC_LEN 200                     // define the maximum number of received bytes
#define USART_TX_LEN 200                      // define the maximum number of sending bytes
#define RevievCharNum 16                      // correct recieveing char number exclude terminator
#define PosSign 1                             // positive sign
#define NegSign 0                             // negative sign
extern char USART_RX_BUF[USART_REC_LEN];      // receiving buffer
extern int USART_RX_STA;                      // recieving number flag
extern bool receiveCompleted;                 // receiving completing flag
extern bool receiveContinuing;                // receiving continuous flag to avoid the multiple data format in buffer: xx\r\n+xx\r\n+xx...
extern bool SendPC_update;                    // data sending to PC enable flag
extern char SwitchFlag;                       // mark if new command have recieved before sending data to PC
extern char USART_TX_BUF[USART_TX_LEN];       // sending buffer
extern int USART_TX_STA;                      // sending number flag
extern bool SendItemFlag[14];                  // for convinient of adjust feedback item 
/******************************** Communication receiving data definition ******************************/
extern char inChar1;
extern char inChar2;



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
void receiveDatafromPC(void);

/**
 * Split the data from PC to number
 * @ PC to MCU Protocol: TLxxxxTRxxxxMxxx\r\n (0x0D,0x0A)
 */
void receivedDataPro(void);

/**
 * @ MCU to PC protocol: MxTLxxxxxALxxxxxVLxxxxxTRxxxxxARxxxxxVRxxxxxPxxxxxYxxxxxVxxxxxCLxxxxCRxxxxDLxxxxDRxxxxSxxx\r\n
 * TL/Rxxxxx: (Nm) Torque feedback for left/right transmission system 
 *                 first number indicate sign: 0 for -, 1 for +
 * AL/Rxxxxx: (deg) Hip angle feedback from potentiometer
 *                  first number indicate sign: 0 for -, 1 for +
 * VL/Rxxxxx: (deg/s) Hip angular velocity feedback
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
