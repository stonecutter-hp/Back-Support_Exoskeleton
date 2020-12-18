/***********************************************************************
 * IMU data collectiong function
 * System model parameters
 **********************************************************************/

#ifndef __IMU_H___
#define __IMU_H___

#include "IIC.h"

/**************************************** IMU parameters definition ********************************/
#define rollChan  0
#define pitchChan 1
#define yawChan   2
#define AddrIMUA  0x50
#define AddrIMUB  0x51
#define AddrIMUC  0x52
#define IMU_UpdateRate 200            // Hz
#define IMUFilterCycles 6

extern unsigned char angleTempA[6];   // store the raw data get from IMU A(addr 0x50)
extern unsigned char angleTempB[6];   // store the raw data get from IMU B(addr 0x51)
extern unsigned char angleTempC[6];   // store the raw data get from IMU C(addr 0x52)
extern unsigned char velTempA[6];     // store the raw data get from IMU A(addr 0x50)
extern unsigned char velTempB[6];     // store the raw data get from IMU B(addr 0x51)
extern unsigned char velTempC[6];     // store the raw data get from IMU C(addr 0x52)

extern float angleActualA[3];         // store the angle of IMU A(addr 0x50) for calculation
extern float angleActualB[3];         // store the angle of IMU B(addr 0x51) for calculation
extern float angleActualC[3];         // store the angle of IMU C(addr 0x52) for calculation
extern float angleActual_p[3][3];     // store the last time angle of IMU C(addr 0x52) for calculation of velocity

extern float velActualA[3];           // store the velocity of IMU A(addr 0x50)
extern float velActualB[3];           // store the velocity of IMU B(addr 0x51)
extern float velActualC[3];           // store the velocity of IMU C(addr 0x52)

extern double IMUA_value_filtered[3][IMUFilterCycles];  // store data from IMUA for filtering
extern double IMUB_value_filtered[3][IMUFilterCycles];  // store data from IMUB for filtering
extern double IMUC_value_filtered[3][IMUFilterCycles];  // store data from IMUC for filtering
extern double IMU_value_Prev[3][3];                     // store last time angle of 3 IMU * 3 channel

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
 * Get euler angle from third IMU
 */
void getIMUangleT(void);

/**
 * Get euler angle from all IMUs
 */
void getIMUangle(void);

/**
 * Get angular velocity from first IMU
 */
void getIMUvelL(void);

/**
 * Get angular velocity from second IMU
 */
void getIMUvelR(void);

/**
 * Get angular velocity from third IMU
 */
void getIMUvelT(void);

/**
 * Get angular velocity from all IMUs
 */
void getIMUvel(void);

/**
 * Mean Moving filter for the IMUA feedback
 * @param int - cycles: 1~IMUFilterCycles
 * @param int - channel: rollChan/pitchChan/yawChan
 */
void MovingAverFilterIMUA(int cycles, int channel);

/**
 * Mean Moving filter for the IMUB feedback
 * @param int - cycles: 1~IMUFilterCycles
 * @param int - channel: rollChan/pitchChan/yawChan
 */
void MovingAverFilterIMUB(int cycles, int channel);

/**
 * Mean Moving filter for the IMUC feedback
 * @param int - cycles: 1~IMUFilterCycles
 * @param int - channel: rollChan/pitchChan/yawChan
 */
void MovingAverFilterIMUC(int cycles, int channel);


#endif