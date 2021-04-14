/***********************************************************************
 * IMU data collectiong function
 * System model parameters
 **********************************************************************/

#include "IMU.h"

/**************************************** IMU parameters definition ********************************/
unsigned char angleTempA[6];   // store the raw data get from IMU A(addr 0x50)
unsigned char angleTempB[6];   // store the raw data get from IMU B(addr 0x51)
unsigned char angleTempC[6];   // store the raw data get from IMU C(addr 0x52)
unsigned char velTempA[6];     // store the raw data get from IMU A(addr 0x50)
unsigned char velTempB[6];     // store the raw data get from IMU B(addr 0x51)
unsigned char velTempC[6];     // store the raw data get from IMU C(addr 0x52)

float angleActualA[3];         // store the angle of IMU A(addr 0x50) for calculation
float angleActualB[3];         // store the angle of IMU B(addr 0x51) for calculation
float angleActualC[3];         // store the angle of IMU C(addr 0x52) for calculation
float angleActual_p[3][3];     // store the last time angle of IMU C(addr 0x52) for calculation of velocity

float velActualA[3];           // store the velocity of IMU A(addr 0x50)
float velActualB[3];           // store the velocity of IMU B(addr 0x51)
float velActualC[3];           // store the velocity of IMU C(addr 0x52)

double IMUA_value_filtered[3][IMUFilterCycles];  // store data from IMUA for filtering
double IMUB_value_filtered[3][IMUFilterCycles];  // store data from IMUB for filtering
double IMUC_value_filtered[3][IMUFilterCycles];  // store data from IMUC for filtering
double IMU_value_Prev[3][3];                     // store last time filtered angle of 3 IMU * 3 channel

unsigned char yaw2zero[2] = {0x04, 0x00};  // The command for IMU yaw angle return to zero (only for 6-axis algorithm)
/**
 * IMU angle feedback return to zero
 */
void IMU_Init(void)  {
  for(int i=0;i<3;i++) {
    angleActualA[i] = 0;
    angleActualB[i] = 0;
    angleActualC[i] = 0;
    velActualA[i] = 0;
    velActualB[i] = 0;
    velActualC[i] = 0;


    for(int t=0; t<IMUFilterCycles; t++) {
      IMUA_value_filtered[i][t] = 0;
      IMUB_value_filtered[i][t] = 0;
      IMUC_value_filtered[i][t] = 0;
    }
    for(int j=0; j<3; j++) {
      IMU_value_Prev[i][j] = 0;
      angleActual_p[i][j] = 0;
    }
  }
  for(int i=0;i<6;i++) {
    angleTempA[i] = 0;
    angleTempB[i] = 0;
    angleTempC[i] = 0;
    velTempA[i] = 0;
    velTempB[i] = 0;
    velTempC[i] = 0;
  }
}

/**
 * Get euler angle from first IMU
 */
void getIMUangleL(void) {
  IICreadBytes(AddrIMUA,0x3d,6,&angleTempA[0]);  // read angle from IMUA
  // transfer cahr format to float format for the convenience of calculation
  // angleActual_p[1][rollChan] = angleActualA[rollChan];
  // angleActual_p[1][pitchChan] = angleActualA[pitchChan];
  // angleActual_p[1][yawChan] = angleActualA[yawChan];
  angleActualA[rollChan] = (float)CharToShort(&angleTempA[0])/32768*180;   // roll A
  angleActualA[pitchChan] = (float)CharToShort(&angleTempA[2])/32768*180;  // pitch A
  angleActualA[yawChan] = (float)CharToShort(&angleTempA[4])/32768*180;    // yaw A
}

/**
 * Get angular velocity from first IMU
 */
void getIMUvelL(void) {
  IICreadBytes(AddrIMUA,0x37,6,&velTempA[0]);  // read angular velocity from IMUA 
  velActualA[rollChan] = (float)CharToShort(&velTempA[0])/32768*2000;   // roll A
  velActualA[pitchChan] = (float)CharToShort(&velTempA[2])/32768*2000;  // pitch A
  velActualA[yawChan] = (float)CharToShort(&velTempA[4])/32768*2000;    // yaw A  
}

/**
 * Yaw angle set to zero for first IMU
 * Can only be set to zero under 6-axis mode
 */
void set2zeroL(void) {
  IICwriteBytes(AddrIMUA,0x01,2,&yaw2zero[0]);
}

/**
 * Get euler angle from second IMU
 */
void getIMUangleR(void) {
  IICreadBytes(AddrIMUB,0x3d,6,&angleTempB[0]);  // read angle from IMUA
  // transfer cahr format to float format for the convenience of calculation
  // update last time's angle feedback
  // angleActual_p[2][rollChan] = angleActualB[rollChan];
  // angleActual_p[2][pitchChan] = angleActualB[pitchChan];
  // angleActual_p[2][yawChan] = angleActualB[yawChan];

  angleActualB[rollChan] = (float)CharToShort(&angleTempB[0])/32768*180;   // roll A
  angleActualB[pitchChan] = (float)CharToShort(&angleTempB[2])/32768*180;  // pitch A
  angleActualB[yawChan] = (float)CharToShort(&angleTempB[4])/32768*180;    // yaw A
}

/**
 * Get angular velocity from second IMU
 */
void getIMUvelR(void) {
  IICreadBytes(AddrIMUB,0x37,6,&velTempB[0]);  // read angular velocity from IMUB
  velActualB[rollChan] = (float)CharToShort(&velTempB[0])/32768*2000;   // roll B
  velActualB[pitchChan] = (float)CharToShort(&velTempB[2])/32768*2000;  // pitch B
  velActualB[yawChan] = (float)CharToShort(&velTempB[4])/32768*2000;    // yaw B  
}

/**
 * Yaw angle set to zero for second IMU
 * Can only be set to zero under 6-axis mode
 */
void set2zeroR(void) {
  IICwriteBytes(AddrIMUB,0x01,2,&yaw2zero[0]);
}

/**
 * Get euler angle from third IMU
 */
void getIMUangleT(void) {
  IICreadBytes(AddrIMUC,0x3d,6,&angleTempC[0]);  // read angle from IMUA
  // transfer cahr format to float format for the convenience of calculation
  // update last time's angle feedback
  angleActual_p[3][rollChan] = angleActualC[rollChan];
  angleActual_p[3][pitchChan] = angleActualC[pitchChan];
  angleActual_p[3][yawChan] = angleActualC[yawChan];

  angleActualC[rollChan] = (float)CharToShort(&angleTempC[0])/32768*180;   // roll A
  angleActualC[pitchChan] = (float)CharToShort(&angleTempC[2])/32768*180;  // pitch A
  angleActualC[yawChan] = (float)CharToShort(&angleTempC[4])/32768*180;    // yaw A
}

/**
 * Get angular velocity from third IMU
 */
void getIMUvelT(void) {
  IICreadBytes(AddrIMUC,0x37,6,&velTempC[0]);  // read angular velocity from IMUC 
  velActualC[rollChan] = (float)CharToShort(&velTempC[0])/32768*2000;   // roll C
  velActualC[pitchChan] = (float)CharToShort(&velTempC[2])/32768*2000;  // pitch C
  velActualC[yawChan] = (float)CharToShort(&velTempC[4])/32768*2000;    // yaw C  
}

/**
 * Yaw angle set to zero for third IMU
 * Can only be set to zero under 6-axis mode
 */
void set2zeroT(void) {
  IICwriteBytes(AddrIMUC,0x01,2,&yaw2zero[0]);
}

/**
 * Get euler angle from all IMUs
 */
void getIMUangle(void) {
  getIMUangleL();
  delay(1);
  getIMUangleR();
  delay(1);
  getIMUangleT();
}

/**
 * Get angular velocity from all IMUs
 */
void getIMUvel(void) {
  getIMUvelL();
  delay(1);
  getIMUvelR();
  delay(1);
  getIMUvelT();
  delay(1);
}


/**
 * Mean Moving filter for the IMUA feedback
 * @param int - cycles: 1~IMUFilterCycles
 * @param int - channel: rollChan/pitchChan/yawChan
 */
void MovingAverFilterIMUA(int cycles, int channel) {
  float interValue;
  interValue = angleActualA[channel];
  // get this times filtered results
  angleActualA[channel] = IMU_value_Prev[1][channel] + (angleActualA[channel]-IMUA_value_filtered[channel][0])/cycles;
  // update the data in the moving window
  for(int j=0; j<cycles-1; j++) {
    IMUA_value_filtered[channel][j] = IMUA_value_filtered[channel][j+1];
  }
  IMUA_value_filtered[channel][cycles-1] = interValue;
  // store this time's results for next calculation
  IMU_value_Prev[1][channel] = angleActualA[channel];
}

/**
 * Mean Moving filter for the IMUB feedback
 * @param int - cycles: 1~IMUFilterCycles
 * @param int - channel: rollChan/pitchChan/yawChan
 */
void MovingAverFilterIMUB(int cycles, int channel) {
  float interValue;
  interValue = angleActualB[channel];
  // get this times filtered results
  angleActualB[channel] = IMU_value_Prev[2][channel] + (angleActualB[channel]-IMUB_value_filtered[channel][0])/cycles;
  // update the data in the moving window
  for(int j=0; j<cycles-1; j++) {
    IMUB_value_filtered[channel][j] = IMUB_value_filtered[channel][j+1];
  }
  IMUB_value_filtered[channel][cycles-1] = interValue;
  // store this time's results for next calculation
  IMU_value_Prev[2][channel] = angleActualB[channel];
}

/**
 * Mean Moving filter for the IMUC feedback
 * @param int - cycles: 1~IMUFilterCycles
 * @param int - channel: rollChan/pitchChan/yawChan
 */
void MovingAverFilterIMUC(int cycles, int channel) {
  float interValue;
  interValue = angleActualC[channel];
  // get this times filtered results
  angleActualC[channel] = IMU_value_Prev[3][channel] + (angleActualC[channel]-IMUC_value_filtered[channel][0])/cycles;
  // update the data in the moving window
  for(int j=0; j<cycles-1; j++) {
    IMUC_value_filtered[channel][j] = IMUC_value_filtered[channel][j+1];
  }
  IMUC_value_filtered[channel][cycles-1] = interValue;
  // store this time's results for next calculation
  IMU_value_Prev[3][channel] = angleActualC[channel];
}