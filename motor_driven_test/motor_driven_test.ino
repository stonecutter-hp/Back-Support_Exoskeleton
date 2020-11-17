#include <AD7173.h>  // the library for ADC
#include "IIC.h"    // IIC library for IMU

/*  Program try 1 for motor driven test for the signal collection of system identification experiments
Version: 1.0 starts from 20200901

***Program logic***
1) Recieving command from serial port from PC for current/speed control of motor
2) Adjust PWM command along with the variation of command
3) Send feedback from driver, load cell, potentiometer and IMU to PC

Remark:
1) Recieved command is motor current
2) Then calculated necessary PWM duty cycle theoratically
*/


/* -------------------------------------- Parameters Definition ----------------------------------- */

/**************************************** ADC pin definition ********************************/
// Ch0~Ch3 is for load cell
// Ch4 is for battery voltage detection with attenuation ratio 1:11
// Ch5~Ch9 is for motor driver signal detection with attenuation ratio 1:2
// Ch10~Ch15 is for potentiometer with no attenuation
// --------------------- CH0 CH1 CH2 CH3   CH4   CH5 CH6 CH7 CH8 CH9 CH10 CH11 CH12 CH13 CH14 CH15
const int attRatio[16] = {1,  1,  1,  1,  22.5,   2,  2,  2,  2,  2,   1,   1,   1,   1,   1,   1};
#define ADC_SS PA4        //SPI1_NSS-CS 
#define ADC_MISO PA6      //SPI1_MISO-DOUT/RDY
#define LED0 PB0          //PB0-green light
#define LED1 PB1          //PB1
#define ENABLED_CH 16     //sum of ADC Channels
#define MotorCurrL 6      //ADC channel assigned for left motor current feedback
#define MotorCurrR 8      //ADC channel assigned for right motor current feedback
#define MotorVeloL 5      //ADC channel assigned for left motor velocity feedback
#define MotorVeloR 7      //ADC channel assigned for right motor velocity feedback
#define PotentioLP1 10    //ADC channel assigned for left potentiometer 1(P1)
#define PotentioLP2 11    //ADC channel assigned for left potentiometer 2(P2)
#define PotentioRP3 12    //ADC channel assigned for right potentiometer 1(P3)
#define PotentioRP4 13    //ADC channel assigned for right potentiometer 2(P4)
/* ADC conversion data and STATUS register */
byte ADC_data[ENABLED_CH][4];  // store raw data from ADC
bool ADC_update = true;        // ADC_update enable flag 
unsigned long ADC_value[ENABLED_CH];    // store ADC value in long format  
double Aver_ADC_value[ENABLED_CH];   // store the transferred ADC value for calculation
#define FilterCycles 10
double Aver_ADC_value_filtered[ENABLED_CH][FilterCycles];
double Aver_ADC_value_Prev[ENABLED_CH];
/* load cell force transfer */
double LoadCell[4];      // store the transferred force value

/*********************************** Serial communication definition ************************************/
#define USART_REC_LEN 200   // define the maximum number of received bytes
#define USART_TX_LEN 200   // define the maximum number of sending bytes
char USART_RX_BUF[USART_REC_LEN];   // receiving buffer
int USART_RX_STA = 0;    // recieving number flag
bool receiveCompleted = false;  // receiving completing flag
bool receiveContinuing = false;  // receiving continuous flag to avoid the multiple data format in buffer: xx\r\n+xx\r\n+xx...
bool SendPC_update = true;      // data sending to PC enable flag
char USART_TX_BUF[USART_TX_LEN];   // sending buffer
int USART_TX_STA = 0;   // sending number flag
int Control_Cycle = 1000;  // controlled sending cycles
int Sending_cycle_flag = 0;  // controlled sending cycle flag
bool Sending_mode = false;  // sending mode false is continuing sending while mode true is recieving enable sending

/*********************************** Communication receiving data definition ************************************/
float desiredTorqueL;    // desired motor torque of left motor
float desiredTorqueR;    // desired motor torque of right motor
int inChar;
/*********************************** Timer configuration parameter definition ************************************/
// Timer3(CH4) is assigned for ADC and Control(PWM) command frequency arrangement
// Timer4(CH3)(PB8) is assigned for frequency of data sending to PC
// Notice the Overflow value should range in 0~65535
#define TIM3_CH4 4   // timer3 channel4
#define TIM3preScale 72   // 72MHz/72 = 1Mhz
#define TIM3_OverflowValue 1000  // 1Mhz/1000 = 1kHz
#define TIM4_CH3 3   // timer4 channel3
#define TIM4preScale 72  // 72MHz/72 = 1MHz
#define TIM4_OverflowValue 4500  // 1Mhz/5000 = 200Hz

/**************************************** PWM related timers parameter definition ********************************/
// Timer1_CH1(PA8) is assigned for the first channel PWM for left motor
// Timer2_CH2(PA1) is assigned for the seconed channel PWM for right motor
#define MotorPWM_L PA8    // pin for left PWM
#define MotorPWM_R PA1    // pin for right PWM
#define TIM1_CH1 1  // timer1 channel1
#define TIM2_CH2 2  // timer2 channel2
// reminder the value should not exceed 65535
// the PWM pulse value is define in control part
uint16 PWMperiod_L = 500;    // period of left PWM  
uint16 PWMperiod_R = 500;    // period of right PWM  

/**************************************** Low level PID control parameters definition ********************************/
// Here use increment PID algorithm: Delta.U = Kp*( (ek-ek_1) + (Tcontrol/Ti)*ek + (Td/Tcontrol)*(ek+ek_2-2*ek_1) )
typedef struct
{
  float currT;  //current torque feedback
  float set;    //deseried torque

  float Err;    //error
  float Err_p;  // last time error
  float Err_pp; // last last time error
  
  float Kp;       // Propotional coefficient
  float Ti;       // Time constant of integration part
  float Td;       // Time constant of derivative part
  float Tcontrol; // Control period, every Tcontrol MCU update once output command
  
  float Delta_PWM;    // Calculation results
  float currpwm;    // Current PWM duty width
  uint16_t pwm_cycle;  // The whole period of PWM
}PID;    // PID parameter structure

PID pidL;  // control parameter of left motor
PID pidR;  // control parameter of right motor
// the desired torque from PC is defined in communication receiving parameter part
uint16_t PWM_commandL;   // range: 0.1*PWMperiod_L~0.9*PWMperiod_L
uint16_t PWM_commandR;   // range: 0.1*PWMperiod_R~0.9*PWMperiod_R
bool Control_update = true;  // control update flag


/**************************************** Actuation unit parameters definition ********************************/
// The output ability of the actuation unit with 19:1 gear ratio is better restricted to 0~8.9 Nm (0.0525*9*19)
#define MotorCurrentConstant 0.0525   //motor current constant Nm/A
#define MotorMaximumCurrent 9        //motor maximum current A configured in EXCON studio
#define GearRatio 19                //gear ratio is 19:1

/**************************************** IMU parameters definition ********************************/
unsigned char angleTempA[6];   // store the raw data get from IMU A(addr 0x50)
unsigned char angleTempB[6];   // store the raw data get from IMU B(addr 0x51)
float angleActualA[3];         // store the angle of IMU A(addr 0x50) for calculation
float angleActualB[3];         // store the angle of IMU B(addr 0x51) for calculation


/* ---------------------------------------- Program ------------------------------------------ */
void setup() {
  /******************************** Serial Initialization ******************************/
  Serial.begin(460800);   // initialize serial set baurd rate
  
  /*************************** Control parameter Initialization ************************/
  Control_Init();  // initialize the control parameters
  
  /******************************* ADC Initialization **********************************/
  afio_cfg_debug_ports(AFIO_DEBUG_NONE);// Stop the debug function
  // initialize the digital pin as an output.
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(ADC_SS, OUTPUT);   
  digitalWrite(ADC_SS, LOW); //enable this device
  /* initialize SPI connection to the ADC */
  AD7173.init();
  /* reset the ADC registers to default */
  AD7173.reset();
  /* set ADC input channel configuration */
  /* enable channel 0 and channel 1 and connect each to 2 analog inputs for bipolar input ????*/ 
  /* CH0 - CH15 */
  /* true/false to enable/disable channel */
  /* SETUP0 - SETUP7 */
  /* AIN0 - AIN16 */
  AD7173.set_channel_config(CH0, false, SETUP0, AIN0, REF_NEG);
  AD7173.set_channel_config(CH1, false, SETUP0, AIN1, REF_NEG);
  AD7173.set_channel_config(CH2, false, SETUP0, AIN2, REF_NEG);
  AD7173.set_channel_config(CH3, false, SETUP0, AIN3, REF_NEG);
  AD7173.set_channel_config(CH4, false, SETUP0, AIN4, REF_NEG);
  AD7173.set_channel_config(CH5, true, SETUP0, AIN5, REF_NEG);
  AD7173.set_channel_config(CH6, true, SETUP0, AIN6, REF_NEG);
  AD7173.set_channel_config(CH7, false, SETUP0, AIN7, REF_NEG);
  AD7173.set_channel_config(CH8, false, SETUP0, AIN8, REF_NEG);
  AD7173.set_channel_config(CH9, false, SETUP0, AIN9, REF_NEG);
  AD7173.set_channel_config(CH10, true, SETUP0, AIN10, REF_NEG);
  AD7173.set_channel_config(CH11, false, SETUP0, AIN11, REF_NEG);
  AD7173.set_channel_config(CH12, false, SETUP0, AIN12, REF_NEG);
  AD7173.set_channel_config(CH13, false, SETUP0, AIN13, REF_NEG);
  AD7173.set_channel_config(CH14, false, SETUP0, AIN14, REF_NEG);
  AD7173.set_channel_config(CH15, false, SETUP0, AIN15, REF_NEG);
  /* set the ADC SETUP0 coding mode to UNIPOLAR output */
  /* SETUP0 - SETUP7 */
  /* BIPOLAR, UNIPOLAR */
  AD7173.set_setup_config(SETUP0, UNIPOLAR, AIN_BUF_ENABLE, REF_INT);

  /* set ADC OFFSET0 offset value */
  /* OFFSET0 - OFFSET7 */
  //AD7173.set_offset_config(OFFSET0, 0);

  /* set the ADC FILTER0 ac_rejection to false and samplingrate to 1007 Hz */
  /* FILTER0 - FILTER7 */
  /* SPS_1, SPS_2, SPS_5, SPS_10, SPS_16, SPS_20, SPS_49, SPS_59, SPS_100, SPS_200 */
  /* SPS_381, SPS_503, SPS_1007, SPS_2597, SPS_5208, SPS_10417, SPS_15625, SPS_31250 */
  AD7173.set_filter_config(FILTER0, SPS_2597);

  /* set the ADC data and clock mode */
  /* CONTINUOUS_CONVERSION_MODE, SINGLE_CONVERSION_MODE */
  /* in SINGLE_CONVERSION_MODE after all setup channels are sampled the ADC goes into STANDBY_MODE */
  /* to exit STANDBY_MODE use this same function to go into CONTINUOUS or SINGLE_CONVERSION_MODE */
  /* INTERNAL_CLOCK, INTERNAL_CLOCK_OUTPUT, EXTERNAL_CLOCK_INPUT, EXTERNAL_CRYSTAL */
  AD7173.set_adc_mode_config(CONTINUOUS_CONVERSION_MODE, INTERNAL_CLOCK, REF_ENABLE);
  
  /* enable/disable CONTINUOUS_READ_MODE and appending STATUS register to data */
  /* to exit CONTINUOUS_READ_MODE use AD7173.reset(); */
  /* AD7173.reset(); returns all registers to default state, so everything has to be setup again */
  AD7173.set_interface_mode_config(false, true);
  
  /* wait for ADC */
  delay(10);

  digitalWrite(LED0,HIGH);  // finish setup and light the green light

// Initial the matrix for average filter Aver_ADC_value_filtered[ENABLED_CH][FilterCycles]
  for(int i=0; i<ENABLED_CH; i++) {
    for(int j=0; j<FilterCycles; j++) {
      Aver_ADC_value_filtered[i][j] = 0.0;
    }
    Aver_ADC_value_Prev[i] = 0.0;
    Aver_ADC_value[i] = 0.0;
  }
  
  
  /******************************** IIC Initialization ******************************/
  IIC_Init();   //IIC initialization for IMU communication
  
  /******************************** Timer Initialization ******************************/
//  Timer3.init();   // stop the timers before configuring them
//  Timer4.init();   // stop the timers before configuring them
//  // Configurate timer 3    1khz
//  Timer3.setPrescaleFactor(TIM3preScale);      // set pre scale factor 72 -->  72MHz/72=1MHz
//  Timer3.setOverflow(TIM3_OverflowValue);      // set overflow value to determine timer 3 frequency: 1/overflow
//  Timer3.setCompare(TIM3_CH4,TIM3_OverflowValue);  // set compare value within overflow for frequency of timers interrupt
//  Timer3.attachInterrupt(TIM3_CH4,Timer3_4_int);   // attach interrupt to timer3 channel4
//  Timer3.refresh();                            // refresh the timer 3 configuration
//  // Configurate timer 4    200hz
//  Timer4.setPrescaleFactor(TIM4preScale);     // set pre scale factor 72 -->  72MHz/72=1MHz
//  Timer4.setOverflow(TIM4_OverflowValue);     // set overflow value to determine timer 4 frequency: 1/overflow
//  Timer4.setCompare(TIM4_CH3,TIM4_OverflowValue);   // set compare value within overflow for frequency of timers interrupt
//  Timer4.attachInterrupt(TIM4_CH3,Timer4_3_int);  // attach interrupt to timer4 channel3
//  Timer4.refresh();                           // refresh the timer 4 configuration


  /******************************** PWM Initialization ******************************/
  Timer1.pause();   // stop the timers before configuring them  (?)
//  Timer2.init();   // stop the timers before configuring them
  pinMode(MotorPWM_L,PWM);    // assign PWM pin mode
//  pinMode(MotorPWM_R,PWM);
  // Configuration timer1 channel1  2khz
  Timer1.setPrescaleFactor(72);   // counting frequency = 72/72 = 1MHz 
  Timer1.setOverflow(PWMperiod_L);        // counting 500 with 1MHz = 0.5ms  2kHz
  Timer1.setCompare(TIM1_CH1,PWM_commandL);
  Timer1.refresh();  
  // Configuration timer2 channel2  2khz
//  Timer2.setPrescaleFactor(72);   // counting frequency = 72/72 = 1MHz 
//  Timer2.setOverflow(PWMperiod_R);        // counting 500 with 1MHz = 0.5ms  2kHz
//  Timer2.setCompare(TIM2_CH2,PWM_commandR);
//  Timer2.refresh(); 

  // resume all the timers
  Timer1.resume();
//  Timer2.resume();  
//  Timer3.resume();
//  Timer4.resume();
}


void loop() {
  receiveDatafromPC();    // receive data from PC
  receivedDataPro();      // decomposite data received from PC
  getADCaverage(1);       // get ADC value
//  MovingAverageFilter(2);  // Averaged moving filtered
  Control(2);             // calculate controlled command: PWM duty cycles
  sendDatatoPC();         // send sensor data to PC and allow next receiving cycle   
}


/* ---------------------------------------------Functional functions---------------------------------------------- */

/************************************* Receive data from PC *********************************************/
/*
 * PC to MCU Protocol: xx.xxxX\r 
 * The final digital is for motor enable control, 1 is enable, 0 is disable
 */
void receiveDatafromPC() {
  while(Serial.available() && receiveContinuing) {
    inChar = Serial.read();
    delayMicroseconds(20);
    // the string should end with \r
    if(inChar == '\r') {
      receiveCompleted = true;  // correct receiving cycle
      receiveContinuing = false;   //finish this receiving cycle
    }
    else {
      if(USART_RX_STA < USART_REC_LEN) {
        USART_RX_BUF[USART_RX_STA] = inChar;
        USART_RX_STA++;
      }
      // exceed the buffer size
      else {
        USART_RX_STA = 0;
        receiveCompleted = false;    // incorrect receiving cycle
        receiveContinuing = false;   //finish this receiving cycle
      }
    }
  }
  
}


/************************************* split the data from PC to numbers *********************************************/
/*
 * PC to MCU Protocol: xx.xxxX\r
 */

void receivedDataPro() {
  // remind that the recieved data are stored in receiving buffer USART_RX_BUF[0~USART_RX_STA]: xx.xxx
  // if the receiving cycle is correct completed
  // here is actuatlly the desired current command recieved from PC for current control testing
  if(receiveCompleted) {
    desiredTorqueL = (USART_RX_BUF[0]-48)*10+(USART_RX_BUF[1]-48)*1+(USART_RX_BUF[3]-48)*0.1+(USART_RX_BUF[4]-48)*0.01+(USART_RX_BUF[5]-48)*0.001;
    if((USART_RX_BUF[6]-48) == 1) {
      digitalWrite(PB9,HIGH);
      digitalWrite(LED0,HIGH);
    }
    else {
      digitalWrite(PB9,LOW);
      digitalWrite(LED0,LOW);
    }
    Serial.println(desiredTorqueL);
  }
}


/************************************* send data to PC *********************************************/
/*
 * @ MCU to PC protocol: AxxxxSxxxxRxxxxPxxxxYxxxx\r\n
 */ 
void sendDatatoPC() {
    Aver_ADC_value[MotorCurrL] = Aver_ADC_value[MotorCurrL]*9/4;
    Aver_ADC_value[MotorVeloL] = Aver_ADC_value[MotorVeloL]*1000;
    Serial.print(Aver_ADC_value[MotorCurrL]);
    Serial.print(' ');
    Serial.print(Aver_ADC_value[MotorVeloL]); 
    Serial.print(' ');
    Serial.println(Aver_ADC_value[10]); 
    Serial.flush();   
    
    SendPC_update = false;
    receiveContinuing = true;
    USART_RX_STA = 0;
    receiveCompleted = false;
}

/*
 * calculate m^n
 * @return: the results of m^n
 */
uint32_t Calcu_Pow(uint8_t m,uint8_t n)
{
  uint32_t result=1;  
  while(n--)result*=m;    
  return result;
}

/*
 * @return: the sign of the value
 */
double Value_sign(double data)
{
  if(data >= 0)
    return 1;
  else
    return -1;
}

/************************************* get value from ADC *********************************************/
/*
 * Get the ADC value of certain channels
 * 
 */
void getADC() {
  for(int i=0;i<ENABLED_CH;i++) {
    while(digitalRead(ADC_MISO) == HIGH) {
      } //wait for data
    /* get ADC conversion result */
    AD7173.get_data(ADC_data[i], true);
  }
  // reorder the rank 0~F
  for(int i=0;i<ENABLED_CH;i++) {
    char tempValue[6];  // store the ADC data
    char tempState[2];  // store the ADC status
    for(int j=0;j<3;j++) {
      sprintf(tempValue+2*j, "%.2X", ADC_data[i][j]);
    }
    sprintf(tempState, "%.2X", ADC_data[i][3]);
    if(tempState[1] == '0'){
      ADC_value[0] = strtoul(tempValue,0,16); 
    }
    else if(tempState[1] == '1') {
      ADC_value[1] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == '2') {
      ADC_value[2] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == '3') {
      ADC_value[3] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == '4') {
      ADC_value[4] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == '5') {
      ADC_value[5] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == '6') {
      ADC_value[6] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == '7') {
      ADC_value[7] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == '8') {
      ADC_value[8] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == '9') {
      ADC_value[9] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == 'A') {
      ADC_value[10] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == 'B') {
      ADC_value[11] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == 'C') {
      ADC_value[12] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == 'D') {
      ADC_value[13] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == 'E') {
      ADC_value[14] = strtoul(tempValue,0,16);
    }
    else if(tempState[1] == 'F') {
      ADC_value[15] = strtoul(tempValue,0,16);
    }
  }
}

/* 
 * Get average value from ADC for cycles
 * @para times: the cycles ADC go through before give you the detected value
 */
void getADCaverage(int times) {
  unsigned long tempADCvalue[ENABLED_CH];
  for(int i=0;i<ENABLED_CH;i++) {
    tempADCvalue[i] = 0;
  }
  for(int i=0;i<times;i++) {
    getADC();
    for(int t=0;t<ENABLED_CH;t++) {
      tempADCvalue[t] += ADC_value[t];
    }
  }
  for(int i=0;i<ENABLED_CH;i++) {
    tempADCvalue[i] = tempADCvalue[i]/times;
    Aver_ADC_value[i] = (double)(tempADCvalue[i]*attRatio[i]*2.5)/16777251;  //24 bits
  }
}

/************************************* Moving filter for the ADC *********************************************/
/*
 * Mean Moving filter for the ADC value
 * @param cycles: 1~FilterCycles
 */
 // Initial the matrix for average filter Aver_ADC_value_filtered[ENABLED_CH][FilterCycles]
void MovingAverageFilter(int cycles) {
  for(int i=0;i<2;i++) {
    for(int j=0; j<cycles-1; j++) {  // update the ADC data
      Aver_ADC_value_filtered[5*i+5][j] = Aver_ADC_value_filtered[5*i+5][j+1];
    }
    Aver_ADC_value_filtered[5*i+5][cycles-1] = Aver_ADC_value[5*i+5];
    // get this time's filtered ADC value
    Aver_ADC_value[5*i+5] = Aver_ADC_value_Prev[5*i+5] + (Aver_ADC_value_filtered[5*i+5][cycles-1] - Aver_ADC_value_filtered[5*i+5][0])/cycles;
    Aver_ADC_value_Prev[5*i+5] = Aver_ADC_value[5*i+5];
  }
}

/************************************* Moving filter for the ADC *********************************************/
/*
 * Exponential moving average filter for the ADC value
 * @param cycles: 1~FilterCycles
 */
void ExponentialMovingFilter(double alpha) {
  for(int i=0;i<2;i++) {
    Aver_ADC_value[5*i+5] = alpha*Aver_ADC_value[5*i+5]+(1-alpha)*Aver_ADC_value_Prev[5*i+5];
    Aver_ADC_value_Prev[5*i+5] = Aver_ADC_value[5*i+5];
  }
}
/************************************* get value from IMU *********************************************/
/*
 * Get euler angle from two IMU
 */
void getIMUangle() {
  IICreadBytes(0x50,0x3d,6,&angleTempA[0]);  // read angle from IMUA
  delay(1);
  IICreadBytes(0x51,0x3d,6,&angleTempB[0]);  // read angle from IMUB
  // transfer cahr format to float format for the convenience of calculation
  angleActualA[0] = (float)CharToShort(&angleTempA[0])/32768*180;   // roll A
  angleActualA[1] = (float)CharToShort(&angleTempA[2])/32768*180;   // pitch A
  angleActualA[2] = (float)CharToShort(&angleTempA[4])/32768*180;   // yaw A
  angleActualB[0] = (float)CharToShort(&angleTempB[0])/32768*180;   // roll B
  angleActualB[1] = (float)CharToShort(&angleTempB[2])/32768*180;   // pitch B
  angleActualB[2] = (float)CharToShort(&angleTempB[4])/32768*180;   // yaw B
}


/************************************* lower-level control functions *********************************************/
/*
 * Control parameter initialization
 */
void Control_Init() {
  PWM_commandL = 0;
  PWM_commandR = 0;
  desiredTorqueL = 0;
  desiredTorqueR = 0;
  // initialize the control parameter of left motor
  pidL.set = desiredTorqueL;
  pidL.currpwm=0.1*PWMperiod_L;
  pidL.pwm_cycle=PWMperiod_L;     
  pidL.Kp=5;   // should be adjusted
  pidL.Td=2000;   // should be adjusted, unit:us
  pidL.Ti=4000;   // should be adjusted, unit:us
  pidL.Tcontrol=TIM3_OverflowValue;  // corresopnding to timer3, unit:us while prescale coefficient = 72
  
  pidL.Err = 0;
  pidL.Err_p = 0;
  pidL.Err_pp = 0;

  // initialize the control parameter of right motor
  pidR.set = desiredTorqueR;
  pidR.currpwm=0.1*PWMperiod_R;
  pidR.pwm_cycle=PWMperiod_R;     
  pidR.Kp=5;     // should be adjusted
  pidR.Td=2000;    // should be adjusted, unit:us
  pidR.Ti=4000;    // should be adjusted, unit:us
  pidR.Tcontrol=TIM3_OverflowValue;    // corresopnding to timer3, unit:us while prescale coefficient = 72
  
  pidR.Err = 0;
  pidR.Err_p = 0;
  pidR.Err_pp = 0;
}

/*
 * Calculate control command (PWM duty cycle) accroding to command received from PC
 * and information received from ADC and IMU
 * @para mode: 1-PID control, 2-Open loop control
 */
void Control(uint8_t mode) {
  float dk1L,dk2L;
  float PoutL,IoutL,DoutL;
  float dk1R,dk2R;
  float PoutR,IoutR,DoutR;

  if(mode == 1) {
    /************************ PID control for left motor *************************/
    pidL.set = desiredTorqueL;
    pidL.currT = Aver_ADC_value[MotorCurrL]*MotorCurrentConstant;   // get current toruqe feedback
    pidL.Err = pidL.set - pidL.currT;                        // calculate the error of this time
    // P
    dk1L = pidL.Err - pidL.Err_p;
    PoutL = pidL.Kp*dk1L;
    // I
    IoutL = (pidL.Kp*pidL.Tcontrol)/pidL.Ti;
    IoutL = IoutL*pidL.Err;
    // D
    dk2L = pidL.Err+pidL.Err_pp-2*pidL.Err_p;
    DoutL = (pidL.Kp*pidL.Td)/pidL.Tcontrol;
    DoutL = DoutL*dk2L;
    // calculate the delta value of this time
    pidL.Delta_PWM = PoutL+IoutL+DoutL;
    pidL.currpwm += pidL.Delta_PWM;      //update pwm pulse
    // avoid overflow of PWM
    if(pidL.currpwm > 0.9*pidL.pwm_cycle) {
      pidL.currpwm = 0.9*pidL.pwm_cycle;
    }
    else if(pidL.currpwm < 0.1*pidL.pwm_cycle) {
      pidL.currpwm = 0.1*pidL.pwm_cycle;
    }
    PWM_commandL = pidL.currpwm;
    //update the error
    pidL.Err_pp = pidL.Err_p;
    pidL.Err_p = pidL.Err;

    /************************ PID control for right motor *************************/
    pidR.set = desiredTorqueR;
    pidR.currT = Aver_ADC_value[MotorCurrR]*MotorCurrentConstant;   // get current toruqe feedback
    pidR.Err = pidR.set - pidR.currT;                        // calculate the error of this time
    // P
    dk1R = pidR.Err - pidR.Err_p;
    PoutR = pidR.Kp*dk1R;
    // I
    IoutR = (pidR.Kp*pidR.Tcontrol)/pidR.Ti;
    IoutR = IoutR*pidR.Err;
    // D
    dk2R = pidR.Err+pidR.Err_pp-2*pidR.Err_p;
    DoutR = (pidR.Kp*pidR.Td)/pidR.Tcontrol;
    DoutR = DoutR*dk2R;
    // calculate the delta value of this time
    pidR.Delta_PWM = PoutR+IoutR+DoutR;
    pidR.currpwm += pidR.Delta_PWM;
    // avoid overflow of PWM
    if(pidR.currpwm > 0.9*pidR.pwm_cycle)
    {
      pidR.currpwm = 0.9*pidR.pwm_cycle;
    }
    else if(pidR.currpwm < 0.1*pidR.pwm_cycle)
    {
      pidR.currpwm = 0.1*pidR.pwm_cycle;
    }
    PWM_commandR = pidR.currpwm;
    //update the error
    pidR.Err_pp = pidR.Err_p;
    pidR.Err_p = pidR.Err;
  }
  // Open-loop control
  else if(mode == 2) {
    PWM_commandL = PWMperiod_L*(desiredTorqueL*0.8/MotorMaximumCurrent+0.1);
    if(PWM_commandL >= 0.9*PWMperiod_L) {
      PWM_commandL = 0.9*PWMperiod_L;
    }
    else if(PWM_commandL <= 0.1*PWMperiod_L)
    {
      PWM_commandL = 0.1*PWMperiod_L;
    }
  } 
  // set the pwm duty cycle  
  Timer1.setCompare(TIM1_CH1,PWM_commandL);    
}

/*
 * Set the pwm duty cycle for both of the motors
 * @para PWMcommandL/PWMcommandR: compared value
 *       range in 0~PWMperiod_L/PWMperiod_R
 */
void MotorPWMoutput(uint16_t PWMcommandL, uint16_t PWMcommandR) {
  Timer1.setCompare(TIM1_CH1,PWM_commandL);
  delay(1);
  Timer2.setCompare(TIM2_CH2,PWM_commandR);
}

/************************************* Interrupt handling function of timer3 and timer4 *********************************************/
/*
 * Interruption of timer3 is for ADC update and Control update
 * Frequency = 72/TIM3preScale/TIM3_OverflowValue = 72MHz/72/1000 = 1kHz
 */
//void Timer3_4_int() {
//  if(ADC_update == false) {
//    ADC_update = true;
//  }
//  if(Control_update == false) {
//    Control_update = true;
//  }
//}
//
/*
 * Interruption timer4 is for data sending to PC
 * Frequency = 72/TIM4preScale/TIM4_OverflowValue = 72MHz/72/2500 = 400Hz
 */
void Timer4_3_int() {
  if(SendPC_update == false) {
    SendPC_update = true;
  }
}
