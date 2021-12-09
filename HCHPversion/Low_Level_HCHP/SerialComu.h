/***********************************************************************
 * The Serial Communication definition and processing function
 **********************************************************************/

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "IIC.h"
#include "Arduino.h"
#include "Control.h"
#include "IMU.h"

/*********************************** Serial communication definition **********************************/
#define USART_REC_LEN 200                     // define the maximum number of received bytes
#define USART_TX_LEN 200                      // define the maximum number of sending bytes
#define RevievCharNum 15                      // correct recieveing char number exclude terminator
#define PosSign 1                             // positive sign
#define NegSign 0                             // negative sign
extern char USART_RX_BUF[USART_REC_LEN];      // receiving buffer
extern int USART_RX_STA;                      // recieving number flag
extern bool receiveCompleted;                 // receiving completing flag
extern bool receiveContinuing;                // receiving continuous flag to avoid the multiple data format in buffer: xx\r\n+xx\r\n+xx...
extern bool SendPC_update;                    // data sending to PC enable flag
extern char SwitchFlag;                       // mark if new command have recieved before sending data to PC
// If the sending info package wants to be adjusted online, the sending buffer should be better defined as a
// local variable instead of global variable to avoid remaining info when info package is adjusted from long to short
extern char USART_TX_BUF[USART_TX_LEN];       // sending buffer
extern int USART_TX_STA;                      // sending number flag
extern bool SendItemFlag[15];                 // for convinient of adjust feedback item adjust
/*********************************** Communication receiving data definition **************************/
extern float desiredTorqueL;    // desired assistive torque of left torque transmission system
extern float desiredTorqueR;    // desired assistive torque of right torque transmission system
extern uint8_t mode;            // detected motion mode
extern uint8_t PreMode;         // last time's motion mode
extern uint8_t side;            // another auxiliary indicator for asymmetric and low-level compensation term from high-level control
/********************************************** Motion state definition *******************************/
#define StopState 0
#define ExitState 1
#define Standing 2
#define Walking 3
#define Lowering 4
#define Grasping 5
#define Lifting 6
#define Holding 7

#define NoneAsy 0
#define LeftAsy 1
#define RightAsy 2
#define FricCompFlag 3

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
 *                                      5 (2+3)-right + fricCom
 * Notice: With successful receiving process, USART_RX_STA indicates
 *         total reveived char number exclude '\r\n'; and they are stored
 *         in USART_RX_BUF[0~USART_RX_STA-1], i.e., TLxxxxTRxxxxMxx 
 *         Here only if terminaor is wrongly detected or data length exceed maximum 
 *         allowable length will lead to fail data receiving
 */
void receiveDatafromPC(void);

/**
 * Split the data from PC to number
 * @ PC to MCU Protocol: TLxxxxTRxxxxMxx\r\n (0x0D,0x0A)
 */
void receivedDataPro(void);

/**
 * @ MCU to PC protocol: MxTLxxxxLLxxxxALxxxxxTRxxxxLRxxxxARxxxxxPxxxxxYxxxxxVxxxxxCLxxxxCRxxxxcLxxxxcRxxxxvLxxxxxvRxxxxx\r\n
 * TL/Rxxxx: (Nm) Torque feedback for left/right transmission system 
 * LL/Rxxxx: (N) Load cell feedback for cable force of left/right transmission system
 * AL/Rxxxxx: (deg) Potentiometer feedback for hip angle feedback
 *                  first number indicate sign: 0 for -, 1 for +
 * Pxxxxx: (deg) Pitch angle for trunk
 * Yxxxxx: (deg) yaw angle for trunk
 *               first number indicate sign: 0 for -, 1 for + 
 * Vxxxxx: (deg/s) Pitch angular velocity for trunk
 *                 first number indicate sign: 0 for -, 1 for +
 * CL/Rxxxx: PWM Cycle Duty
 *           first number indicate sign: 0 for -(loosening cable), 1 for +(tighting cable)
 * cL/Rxxxx: (A) Motor Current Feedback from Driver
 * vL/Rxxxxx: (rpm) Motor Velocity Feedback from Driver
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
