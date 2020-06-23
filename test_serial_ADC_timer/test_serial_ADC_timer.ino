#include <AD7173.h>  // the library for ADC
//test1
//test2

/*  Program try 1 for the low-level control of exoskelton
Version: 1.0 starts from 20200615

***Program logic***
1) read desired torque command from PC --> 
2) read sensor feedback including: 
    potentiometer, 
    motor driver, 
    load cell,
    and IMU -->
3) calculate the actual control commmand for motor -->
4) send sensor feedback to PC 
*/


/* -------------------------------------- Parameters Definition ----------------------------------- */

/**************************************** ADC pin definition ********************************/
// Ch0~Ch3 is for load cell
// Ch4 is for battery voltage detection with attenuation ratio 1:11
// Ch5~Ch9 is for motor driver signal detection with attenuation ratio 1:2
// Ch10~Ch15 is for potentiometer with no attenuation
// --------------------- CH0 CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10 CH11 CH12 CH13 CH14 CH15
const int attRatio[16] = {1,  1,  1,  1,  11, 2,  2,  2,  2,  2,   1,   1,   1,   1,   1,   1};


#define ADC_SS PA4        //SPI1_NSS-CS 
#define ADC_MISO PA6      //SPI1_MISO-DOUT/RDY
#define LED0 PB0          //PB0-green light
#define LED1 PB1          //PB1

#define ENABLED_CH 16     //sum of ADC Channels

#define MotorCurrL 5      //ADC channel assigned for left motor current feedback
#define MotorCurrR 6      //ADC channel assigned for left motor current feedback

/* ADC conversion data and STATUS register */
byte ADC_data[ENABLED_CH][4];  // store raw data from ADC
bool ADC_update = true;        // ADC_update enable flag 
unsigned long ADC_value[ENABLED_CH];    // store ADC value in raw number  
double Aver_ADC_value[ENABLED_CH];   // store the transferred ADC value 


/*********************************** Serial communication definition ************************************/
#define USART_REC_LEN 200   // define the maximum number of received bytes
#define USART_TX_LEN 200   // define the maximum number of sending bytes
char USART_RX_BUF[USART_REC_LEN];   // receiving buffer
int USART_RX_STA = 0; // recieving number flag
bool receiveCompleted = false;  // receiving completing flag
bool receiveContinuing = false;  // receiving continuous flag to avoid the multiple data format in buffer: xx\r\n+xx\r\n+xx...
bool SendPC_update = true;      // data sending to PC enable flag
char USART_TX_BUF[USART_TX_LEN];   // sending buffer
int USART_TX_STA = 0; // sending number flag

/*********************************** Communication receiving data definition ************************************/
float desiredTorqueL;    // desired motor torque of left motor
float desiredTorqueR;    // desired motor torque of right motor

/*********************************** Timer configuration parameter definition ************************************/
// Timer3(CH4) is assigned for ADC and Control(PWM) command frequency arrangement
// Timer4(CH3)(PB8) is assigned for frequency of data sending to PC
// Notice the Overflow value should range in 0~65535
#define TIM3_CH4 4   // timer3 channel4
#define TIM3preScale 72   // 72MHz/72 = 1Mhz
#define TIM3_OverflowValue 1000  // 1Mhz/1000 = 1kHz
#define TIM4_CH3 3   // timer4 channel3
#define TIM4preScale 72  // 72MHz/72 = 1MHz
#define TIM4_OverflowValue 2500  // 1Mhz/2500 = 400Hz

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
uint16_t PWM_commandL;   // range: 0~PWMperiod_L
uint16_t PWM_commandR;   // range: 0~PWMperiod_R
bool Control_update = true;  // control update flag

/**************************************** Actuation unit parameters definition ********************************/
#define MotorCurrentConstant 1   //motor current constant
#define MotorMaximumCurrent 1    //motor norminal current


/* ---------------------------------------- Program ------------------------------------------ */

//Initialization of the program
void setup() {
  /******************************** Serial Initialization ******************************/
  Serial.begin(115200);   // initialize serial set baurd rate
  
  /******************************** Control parameter Initialization ******************************/
  Control_Init();  // initialize the control parameters
  
  /******************************** ADC Initialization ******************************/
  afio_cfg_debug_ports(AFIO_DEBUG_NONE);// Stop the debug function
  // initialize the digital pin as an output.
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
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
  AD7173.set_channel_config(CH0, true, SETUP0, AIN0, REF_NEG);
  AD7173.set_channel_config(CH1, true, SETUP0, AIN1, REF_NEG);
  AD7173.set_channel_config(CH2, true, SETUP0, AIN2, REF_NEG);
  AD7173.set_channel_config(CH3, true, SETUP0, AIN3, REF_NEG);
  AD7173.set_channel_config(CH4, true, SETUP0, AIN4, REF_NEG);
  AD7173.set_channel_config(CH5, true, SETUP0, AIN5, REF_NEG);
  AD7173.set_channel_config(CH6, true, SETUP0, AIN6, REF_NEG);
  AD7173.set_channel_config(CH7, true, SETUP0, AIN7, REF_NEG);
  AD7173.set_channel_config(CH8, true, SETUP0, AIN8, REF_NEG);
  AD7173.set_channel_config(CH9, true, SETUP0, AIN9, REF_NEG);
  AD7173.set_channel_config(CH10, true, SETUP0, AIN10, REF_NEG);
  AD7173.set_channel_config(CH11, true, SETUP0, AIN11, REF_NEG);
  AD7173.set_channel_config(CH12, true, SETUP0, AIN12, REF_NEG);
  AD7173.set_channel_config(CH13, true, SETUP0, AIN13, REF_NEG);
  AD7173.set_channel_config(CH14, true, SETUP0, AIN14, REF_NEG);
  AD7173.set_channel_config(CH15, true, SETUP0, AIN15, REF_NEG);
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

  /******************************** Timer Initialization ******************************/
  Timer3.init();   // stop the timers before configuring them
  Timer4.init();   // stop the timers before configuring them
  // Configurate timer 3    1khz
  Timer3.setPrescaleFactor(TIM3preScale);    
  Timer3.setOverflow(TIM3_OverflowValue);     
  Timer3.setCompare(TIM3_CH4,TIM3_OverflowValue);
  Timer3.attachInterrupt(TIM3_CH4,Timer3_4_int);
  Timer3.refresh();
  // Configurate timer 4    400hz
  Timer4.setPrescaleFactor(TIM4preScale); 
  Timer4.setOverflow(TIM4_OverflowValue);    
  Timer4.setCompare(TIM4_CH3,TIM4_OverflowValue);
  Timer4.attachInterrupt(TIM4_CH3,Timer4_3_int);
  Timer4.refresh();  


  /******************************** PWM Initialization ******************************/
  Timer1.pause();   // stop the timers before configuring them  ???
  Timer2.init();   // stop the timers before configuring them
  pinMode(MotorPWM_L,PWM);    // assign PWM pin mode
  pinMode(MotorPWM_R,PWM);
  // Configuration timer1 channel1  2khz
  Timer1.setPrescaleFactor(72);   // counting frequency = 72/72 = 1MHz 
  Timer1.setOverflow(PWMperiod_L);        // counting 500 with 1MHz = 0.5ms  2kHz
  Timer1.setCompare(TIM1_CH1,PWM_commandL);
  Timer1.refresh();  
  // Configuration timer2 channel2  2khz
  Timer2.setPrescaleFactor(72);   // counting frequency = 72/72 = 1MHz 
  Timer2.setOverflow(PWMperiod_R);        // counting 500 with 1MHz = 0.5ms  2kHz
  Timer2.setCompare(TIM2_CH2,PWM_commandR);
  Timer2.refresh(); 

  // resume all the timers
  Timer1.resume();
  Timer2.resume();  
  Timer3.resume();
  Timer4.resume();
}


// Main Loop
void loop() {
  char c[10];
  receiveDatafromPC();    // receive data from PC
  receivedDataPro();      // decomposite data received from PC
  getADCaverage(1);
  Control(1);

  sendDatatoPC();         // send sensor data to PC and allow next receiving cycle
  

/********************* for serial test **************************/  
  if(receiveCompleted) {
//    Serial.println("raw:");
//    for(int t=0;t<USART_RX_STA;t++) {
//      Serial.print(USART_RX_BUF[t]);
//    }
//    Serial.println();
//    Serial.println("number:");
//--------------- this two should be update after control --------------------------------------------------------------------
    USART_RX_STA = 0;
    receiveCompleted = false;
//    dtostrf(desiredTorqueL,2,2,c);  // transfer double to char
//    Serial.println(c);
//    for(int i=0;i<ENABLED_CH;i++) {
//      dtostrf(Aver_ADC_value[i],2,4,c);  // transfer double to char
//      Serial.println(c);
//    }
//    ADC_update = true;
  }
    receiveContinuing = true;
    // ADC_update = true;
}


/* ---------------------------------------------Functional functions---------------------------------------------- */

/************************************* Receive data from PC *********************************************/
/*
Protocol: xxxx\r\n
*/
void receiveDatafromPC() {
  while(Serial.available() && receiveContinuing) {
    char inChar = (char)Serial.read();
    if(inChar == '\n') {
      USART_RX_STA--;
      if(USART_RX_BUF[USART_RX_STA] != '\r') {
        USART_RX_STA = 0;
//        while(Serial.available() && Serial.read());   // release the buffer for next correct receiving cycle
        receiveCompleted = false;  // incorrect receiving cycle
      }
      else {
        receiveCompleted = true;  // correct receiving cycle
      }
    receiveContinuing = false;   //finish this receiving cycle
    }
    else {
      if(USART_RX_STA < USART_REC_LEN) {
        USART_RX_BUF[USART_RX_STA] = inChar;
        USART_RX_STA++;
      }
      else {
        USART_RX_STA = 0;
//        while(Serial.available() && Serial.read());   // release the buffer for next correct receiving cycle
        receiveCompleted = false;  // incorrect receiving cycle
        receiveContinuing = false;   //finish this receiving cycle
      }
    }
  }
}


/************************************* split the data from PC to numbers *********************************************/
/*
 * protocol:
 */

void receivedDataPro() {
  // remind that the recieved data are stored in receiving buffer USART_RX_BUF[0~USART_RX_STA]: xxxx\r
  // '\r' is not removed in receiving buffer
  // if the receiving cycle is correct completed
  if(receiveCompleted) {
    // depends on the communication/receving procotol
    if(USART_RX_BUF[0] == 'D') {
      desiredTorqueL = (USART_RX_BUF[1]-48)*10+(USART_RX_BUF[2]-48)*1+(USART_RX_BUF[3]-48)*0.1+(USART_RX_BUF[4]-48)*0.01;
      //...
    }
    else if(USART_RX_BUF[0] == '-') {
      if(PWM_commandL>=100){
        PWM_commandL -= 100;
      }
      if(PWM_commandR>=100){
        PWM_commandR -= 100;
      }
    }
    else if(USART_RX_BUF[0] == '+') {
      if(PWM_commandL<=400){
        PWM_commandL += 100;
      }
      if(PWM_commandR<=400){
        PWM_commandR += 100;
      }
    }
  }
  Timer1.setCompare(TIM1_CH1,PWM_commandL);
  delay(1);
  Timer2.setCompare(TIM2_CH2,PWM_commandR);
}


/************************************* send data to PC *********************************************/
/*
 * protocol:AxxxxSxxxxRxxxxPxxxxYxxxx\r\n
 */ 
void sendDatatoPC() {
  unsigned int dec;
  unsigned char inter;
  if(SendPC_update == true) {
    for(int i=0; i<5; i++) {
      // USART_TX_BUF[i*5] = 'V';
      // dec = Aver_ADC_value[5+i]*Calcu_Pow(10,3);  // t<4 means here count for 0.001 precision
      if(i==0)
      {
        USART_TX_BUF[0] = 'A';
        dec = Aver_ADC_value[5]*Calcu_Pow(10,3);  // t<4 means here count for 0.001 precision
      }
      else if(i==1)
      {
        USART_TX_BUF[5] = 'S';
        dec = Aver_ADC_value[6]*Calcu_Pow(10,3);  // t<4 means here count for 0.001 precision
      }
      else if(i==2)
      {
        USART_TX_BUF[10] = 'R';
        dec = Aver_ADC_value[7]*Calcu_Pow(10,3);  // t<4 means here count for 0.001 precision
      }
      else if(i==3)
      {
        USART_TX_BUF[15] = 'P';
        dec = Aver_ADC_value[8]*Calcu_Pow(10,3);  // t<4 means here count for 0.001 precision
      }
      else if(i==4)
      {
        USART_TX_BUF[20] = 'Y';
        dec = Aver_ADC_value[9]*Calcu_Pow(10,3);  // t<4 means here count for 0.001 precision
      }      
      for(int t=0;t<4;t++) {
        inter = (dec/Calcu_Pow(10,3-t))%10;              //Seperate every single number 
        USART_TX_BUF[i*5+t+1] = inter+48;
      }
      Serial.print(USART_TX_BUF);
      Serial.flush();
      Serial.print('\r');
      Serial.flush();
      Serial.print('\n');
      Serial.flush();
    }
  }
  SendPC_update = false;
  // after sending the data to PC, allow next receiving cycle
  receiveContinuing = true;
}

//calculate m^n
//reture the results of m^n
uint32_t Calcu_Pow(uint8_t m,uint8_t n)
{
  uint32_t result=1;  
  while(n--)result*=m;    
  return result;
}

//return the sign of the value
double Value_sign(double data)
{
  if(data >= 0)
    return 1;
  else
    return -1;
}

/************************************* get value from ADC *********************************************/
void getADC() {
  for(int i=0;i<ENABLED_CH;i++) {
    while(digitalRead(ADC_MISO) == HIGH) {
      } //wait for data
    /* get ADC conversion result */
    AD7173.get_data(ADC_data[i], true);
  }
  // reorder the rank 0~F
  for(int i=0;i<ENABLED_CH;i++) {
    char tempValue[6];  // store the ADC daata
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

/************************************* get average value from ADC for cycles *********************************************/
/*
 * @para times: the cycles ADC go through before give you the detected value
 */
void getADCaverage(int times) {
  unsigned long tempADCvalue[ENABLED_CH];
  if(ADC_update == true) {
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
  ADC_update = false;  
}

/************************************* lower-level control functions *********************************************/
// control parameter initialization
void Control_Init() {
  PWM_commandL = 0;
  PWM_commandR = 0;
  desiredTorqueL = 0;
  desiredTorqueR = 0;
// initialize the control parameter of left motor
  pidL.set = desiredTorqueL;
  pidL.currpwm=0;
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
  pidR.currpwm=0;
  pidR.pwm_cycle=PWMperiod_R;     
  pidR.Kp=5;     // should be adjusted
  pidR.Td=2000;    // should be adjusted, unit:us
  pidR.Ti=4000;    // should be adjusted, unit:us
  pidR.Tcontrol=TIM3_OverflowValue;    // corresopnding to timer3, unit:us while prescale coefficient = 72
  
  pidR.Err = 0;
  pidR.Err_p = 0;
  pidR.Err_pp = 0;
}

void Control(uint8_t mode) {
  float dk1L,dk2L;
  float PoutL,IoutL,DoutL;
  float dk1R,dk2R;
  float PoutR,IoutR,DoutR;

  if(Control_update == true) {
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

      pidL.Delta_PWM = PoutL+IoutL+DoutL;
      pidL.currpwm += pidL.Delta_PWM;      //update pwm pulse
      if(pidL.currpwm > pidL.pwm_cycle) {
        pidL.currpwm = pidL.pwm_cycle;
      }
      else if(pidL.currpwm < 0) {
        pidL.currpwm = 0;
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
    
      pidR.Delta_PWM = PoutR+IoutR+DoutR;
      pidR.currpwm += pidR.Delta_PWM;
    
      if(pidR.currpwm > pidR.pwm_cycle)
      {
        pidR.currpwm = pidR.pwm_cycle;
      }
      else if(pidR.currpwm < 0)
      {
        pidR.currpwm = 0;
      }
      PWM_commandR = pidR.currpwm;
    
      //update the error
      pidR.Err_pp = pidR.Err_p;
      pidR.Err_p = pidR.Err;
    }
    else if(mode == 2) {
      PWM_commandL = PWMperiod_L*desiredTorqueL/(MotorCurrentConstant*MotorMaximumCurrent);
      PWM_commandR = PWMperiod_R*desiredTorqueR/(MotorCurrentConstant*MotorMaximumCurrent);
      if(PWM_commandL > pidL.pwm_cycle) {
        PWM_commandL = pidL.pwm_cycle;
      }
      else if(PWM_commandL < 0)
      {
        PWM_commandL = 0;
      }
      if(PWM_commandR > pidR.pwm_cycle) {
        PWM_commandR = pidR.pwm_cycle;
      }
      else if(PWM_commandR < 0)
      {
        PWM_commandR = 0;
      }
    }
    MotorPWMoutput(PWM_commandL,PWM_commandR);        
  }
  Control_update = false;     // allow next control cycle
  // mark the data of this receiving cycle is used, wait for another cycle
  USART_RX_STA = 0;    
  receiveCompleted = false;
}


void MotorPWMoutput(uint16_t PWMcommandL, uint16_t PWMcommandR) {
  Timer1.setCompare(TIM1_CH1,PWM_commandL);
  delay(1);
  Timer2.setCompare(TIM2_CH2,PWM_commandR);
}

/************************************* Interrupt handling function of timer3 and timer4 *********************************************/
// timer3 is for ADC update and Control update
void Timer3_4_int() {
  if(ADC_update == false) {
    ADC_update = true;
  }
  if(Control_update == false) {
    Control_update = true;
  }
  
}

// timer4 is for data sending to PC 
void Timer4_3_int() {
  if(SendPC_update == false) {
    SendPC_update = true;
  }
}
