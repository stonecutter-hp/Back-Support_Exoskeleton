/***********************************************************************
 * IMU data collectiong function
 * System model parameters
 **********************************************************************/

#include "IMU.h"

/**************************************** IMU parameters definition ********************************/
unsigned char angleTempA[6];   // store the raw data get from IMU A(addr 0x50)
unsigned char angleTempB[6];   // store the raw data get from IMU B(addr 0x51)
float angleActualA[3];         // store the angle of IMU A(addr 0x50) for calculation
float angleActualB[3];         // store the angle of IMU B(addr 0x51) for calculation

/**
 * IMU angle feedback return to zero
 */
void IMU_Init(void)  {
  for(int i=0;i<3;i++) {
    angleActualA[i] = 0;
    angleActualB[i] = 0;
    angleTempA[i] = 0;
    angleTempB[i] = 0;
  }
}

/**
 * Get euler angle from first IMU
 */
void getIMUangleL(void) {
  IICreadBytes(0x50,0x3d,6,&angleTempA[0]);  // read angle from IMUA
  // transfer cahr format to float format for the convenience of calculation
  angleActualA[0] = (float)CharToShort(&angleTempA[0])/32768*180;   // roll A
  angleActualA[1] = (float)CharToShort(&angleTempA[2])/32768*180;   // pitch A
  angleActualA[2] = (float)CharToShort(&angleTempA[4])/32768*180;   // yaw A
}

/**
 * Get euler angle from second IMU
 */
void getIMUangleR(void) {
  IICreadBytes(0x51,0x3d,6,&angleTempB[0]);  // read angle from IMUA
  // transfer cahr format to float format for the convenience of calculation
  angleActualB[0] = (float)CharToShort(&angleTempB[0])/32768*180;   // roll A
  angleActualB[1] = (float)CharToShort(&angleTempB[2])/32768*180;   // pitch A
  angleActualB[2] = (float)CharToShort(&angleTempB[4])/32768*180;   // yaw A
}

/**
 * Get euler angle from two IMU
 */
void getIMUangle(void) {
	getIMUangleL();
	delay(1);
	getIMUangleR();
}