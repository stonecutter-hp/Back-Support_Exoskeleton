/***********************************************************************
 * IMU data collectiong function
 * System model parameters
 **********************************************************************/

#ifndef __IMU_H___
#define __IMU_H___

#include "IIC.h"

/**************************************** IMU parameters definition ********************************/
extern unsigned char angleTempA[6];   // store the raw data get from IMU A(addr 0x50)
extern unsigned char angleTempB[6];   // store the raw data get from IMU B(addr 0x51)
extern float angleActualA[3];         // store the angle of IMU A(addr 0x50) for calculation
extern float angleActualB[3];         // store the angle of IMU B(addr 0x51) for calculation


/**
 * IMU angle feedback return to zero
 */
void IMU_Init(void);

/**
 * Get euler angle from first IMU
 */
void getIMUangleL(void);

/**
 * Get euler angle from second IMU
 */
void getIMUangleR(void);

/**
 * Get euler angle from two IMU
 */
void getIMUangle(void);


#endif