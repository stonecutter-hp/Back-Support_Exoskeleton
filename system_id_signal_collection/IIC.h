/***********************************************************************
 * The IIC functional functions for communication
 * between MCU(stm32) and IMU (JY901)
 **********************************************************************/

#ifndef _IIC_H__
#define _IIC_H__

#include <libmaple/libmaple_types.h>
#include <boards.h>
#include <io.h>
#include <wirish_time.h>


#define IIC_SCL_1 digitalWrite(PB6,HIGH)
#define IIC_SCL_0 digitalWrite(PB6,LOW)
#define IIC_SDA_1 digitalWrite(PB7,HIGH)
#define IIC_SDA_0 digitalWrite(PB7,LOW)
#define SDA_OUT() {pinMode(PB7,OUTPUT);} 
#define SDA_IN() {pinMode(PB7,INPUT);}
#define READ_SDA digitalRead(PB7)


/**
 * Set the pin as open-drain mode for usage of IIC communication
 */
void IIC_Init(void);

/**
 * Produce IIC start signal
 */
void IIC_Start(void);

/**
 * Produce IIC stop signal
 */
void IIC_Stop(void);

/**
 * Wait for acknowledegment signal
 * 
 * @return receiving status
 *         1: fail in receiving ack signal
 *         0: success in receiving ack signal 
 */
uint8_t IIC_Wait_Ack(void);

/**
 * Produce acknowledegment signal
 */
void IIC_Ack(void);

/**
 * Not produce acknowledegment signal
 */
void IIC_NAck(void);

/**
 * Send one byte data
 * 
 * @param txd Data to be sent
 */
void IIC_Send_Byte(uint8_t txd);

/**
 * Read one byte data
 * 
 * @param ack Send ack signal or not
 *            1, send ack signal
 *            0, senf Nack signal
 * @return data have been received
 */
uint8_t IIC_Read_Byte(unsigned char ack);
 
/**
 * Read certain length of data from certain registers of certain device
 * 
 * @param dev: the address of target device
 *        reg: the address of target register
 *        lengthe: the length of data to read
 *        *data: the point to store the data
 * @return count: the number of byte have read from the register
 */
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t lengthe, uint8_t *data);

/**
 * Write certain length of data from certain registers of certain device
 *
 * @param dev: the address of target device
 *        reg: the address of target register
 *        lengthe: the length of data to write
 *        *data: the point stored the data
 * @return Write status
 *         1: unsuccess, 0: success
 */
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t lengthe, uint8_t* data);

/**
 * Turn data in char format to short format for further calculation
 * 
 * @param cData[] The data waited to be transferred to short format
 * @return The transferred short format data
 */
short CharToShort(unsigned char cData[]);

#endif
