/***********************************************************************
 * The IIC functional functions for communication
 * between MCU(stm32) and IMU (JY901)
 **********************************************************************/

#include "IIC.h"
#include <libmaple/libmaple_types.h>
#include <boards.h>
#include <io.h>
#include <wirish_time.h>

void IIC_Init(void) {
  pinMode(PB6,OUTPUT_OPEN_DRAIN);
  pinMode(PB7,OUTPUT_OPEN_DRAIN); // set as open-drain output mode
  IIC_SDA_1;
  IIC_SCL_1;  
}

void IIC_Start(void) {
  SDA_OUT();  // SDA output
  IIC_SDA_1;
  IIC_SCL_1;
  delayMicroseconds(5);
  IIC_SDA_0;
  delayMicroseconds(5);
  IIC_SCL_0;
}

void IIC_Stop(void) {
  SDA_OUT();  // SDA output
  IIC_SCL_0;
  IIC_SDA_0;  // STOP:when CLK is high DATA change form low to high
  delayMicroseconds(5);
  IIC_SCL_1;
  IIC_SDA_1;
  delayMicroseconds(5);
}

uint8_t IIC_Wait_Ack(void) {
  uint8_t ucErrTime=0;
  SDA_IN();  // set SDA for input
  IIC_SDA_1;delayMicroseconds(4);
  IIC_SCL_1;delayMicroseconds(4);
  while(READ_SDA) {
    ucErrTime++;
    if(ucErrTime>50) {
      IIC_Stop();
      return 1;
    }
  }
  IIC_SCL_0;
  return 0;
}

void IIC_Ack(void) {
  IIC_SCL_0;
  SDA_OUT();
  IIC_SDA_0;
  delayMicroseconds(2);
  IIC_SCL_1;
  delayMicroseconds(2);
  IIC_SCL_0;  
}

void IIC_NAck(void)
{
  IIC_SCL_0;
  SDA_OUT();
  IIC_SDA_1;
  delayMicroseconds(2);
  IIC_SCL_1;
  delayMicroseconds(2);
  IIC_SCL_0;
} 

void IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;   
  SDA_OUT();      
  IIC_SCL_0; //lowering clk to start data transmission
  for(t=0;t<8;t++) {
    if((txd&0x80)>>7) {
      IIC_SDA_1;
    }
    else {
      IIC_SDA_0;
    }
    txd<<=1;    
    delayMicroseconds(2);   //the delay is necessary
    IIC_SCL_1;
    delayMicroseconds(5); 
    IIC_SCL_0;  
    delayMicroseconds(3);
  }  
} 

uint8_t IIC_Read_Byte(unsigned char ack)
{
  unsigned char receive = 0;
  SDA_IN();
  for(int i=0; i<8; i++) {
    IIC_SCL_0;
    delayMicroseconds(5);
    IIC_SCL_1;
    receive<<=1;
    if(READ_SDA) {
      receive++;
    }
    delayMicroseconds(5);
  }
  if(ack) {
    IIC_Ack();
  }
  else {
    IIC_NAck();
  }
  return receive;
}

uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t lengthe, uint8_t *data) {
  uint8_t count = 0;

  IIC_Start();
  IIC_Send_Byte((dev<<1)|0);  // send device address and write command
  IIC_Wait_Ack();     // wait for acknowledge signal
  IIC_Send_Byte(reg);  // send register address
  IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte((dev<<1)|1);  // send device address and read command
  IIC_Wait_Ack();

  for(count=0; count<lengthe; count++) {
    if(count != lengthe-1) {
      data[count] = IIC_Read_Byte(1);
    }
    else {
      data[count] = IIC_Read_Byte(0);  // the last data send Nack signal
    }
  }
  IIC_Stop();
  return count;
}

uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t lengthe, uint8_t* data){
  IIC_Start();
  IIC_Send_Byte((dev<<1)|0);   // send device address and write command
  if(IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);   // send register address
  IIC_Wait_Ack();     // wait for ack signal
  for(int count=0; count<lengthe; count++) {
    IIC_Send_Byte(data[count]);
    if(IIC_Wait_Ack()) {
      IIC_Stop();
      return 1;
    }
  }
  IIC_Stop();
  return 0;  
}

short CharToShort(unsigned char cData[])
{
  return ((short)cData[1]<<8)|cData[0];
}