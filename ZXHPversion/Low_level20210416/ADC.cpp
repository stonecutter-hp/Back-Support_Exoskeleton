/***********************************************************************
 * The ADC pin definition and processing function
 **********************************************************************/

#include "ADC.h"

/* ADC conversion data and STATUS register */
byte ADC_data[ENABLED_CH][4];             // store raw data from ADC
bool ADC_update = true;                   // ADC_update enable flag 
unsigned long ADC_value[ENABLED_CH];      // store ADC value in long format  
double Aver_ADC_value[ENABLED_CH];        // store the transferred ADC value for calculation
double Aver_ADC_value_filtered[ENABLED_CH][FilterCycles];
double Aver_ADC_value_Prev[ENABLED_CH];
/* load cell force transfer */
double LoadCell[4];                      // store the transferred force value


/**
 * ADC initialization for channel mode configuration
 */
void ADC_Init(void) {
  afio_cfg_debug_ports(AFIO_DEBUG_NONE); // Stop the debug function
  pinMode(ADC_SS, OUTPUT);
  digitalWrite(ADC_SS, LOW);         //enable this ADC device
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
  AD7173.set_channel_config(CH0, true, SETUP0, AIN0, REF_NEG);    //LoadCellL
  AD7173.set_channel_config(CH1, false, SETUP0, AIN1, REF_NEG);   
  AD7173.set_channel_config(CH2, false, SETUP0, AIN2, REF_NEG);
  AD7173.set_channel_config(CH3, false, SETUP0, AIN3, REF_NEG);
  AD7173.set_channel_config(CH4, false, SETUP0, AIN4, REF_NEG);
  AD7173.set_channel_config(CH5, false, SETUP0, AIN5, REF_NEG);   //MotorVeloL
  AD7173.set_channel_config(CH6, false, SETUP0, AIN6, REF_NEG);   //MotorCurrL
  AD7173.set_channel_config(CH7, false, SETUP0, AIN7, REF_NEG);
  AD7173.set_channel_config(CH8, false, SETUP0, AIN8, REF_NEG);
  AD7173.set_channel_config(CH9, false, SETUP0, AIN9, REF_NEG);
  AD7173.set_channel_config(CH10, true, SETUP0, AIN10, REF_NEG);  //PotentioLP1
  AD7173.set_channel_config(CH11, false, SETUP0, AIN11, REF_NEG);
  AD7173.set_channel_config(CH12, false, SETUP0, AIN12, REF_NEG); //ForceSensorL
  AD7173.set_channel_config(CH13, false, SETUP0, AIN13, REF_NEG);
  AD7173.set_channel_config(CH14, false, SETUP0, AIN14, REF_NEG);
  AD7173.set_channel_config(CH15, false, SETUP0, AIN15, REF_NEG);
  /* set the ADC SETUP0 coding mode to UNIPOLAR output */
  /* SETUP0 - SETUP7 */
  /* BIPOLAR, UNIPOLAR */
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
}

/**
 * Initial the matrix for average filter Aver_ADC_value_filtered[ENABLED_CH][FilterCycles]
 */
void Filter_Init() {
  for(int i=0; i<ENABLED_CH; i++) {
    for(int j=0; j<FilterCycles; j++) {
      Aver_ADC_value_filtered[i][j] = 0.0;
    }
    ADC_value[i] = 0;
    Aver_ADC_value_Prev[i] = 0.0;
    Aver_ADC_value[i] = 0.0;
    ADC_data[i][3] = 'G';   //Initialize the ADC status flag
  }  
}

/**
 * Mean Moving filter for the ADC value
 * @param int - channel: 0~ENABLED_CH-1
 * @param int - cycles: 1~FilterCycles
 */
void MovingAverageFilter(int channel, int cycles) {
  double interValue;
  if(cycles > FilterCycles) {
    cycles = FilterCycles;
  }
  else if(cycles < 1) {
    cycles = 1;
  }
  interValue = Aver_ADC_value[channel];
  // get this times filtered results
  Aver_ADC_value[channel] = Aver_ADC_value_Prev[channel] + (Aver_ADC_value[channel] - Aver_ADC_value_filtered[channel][0])/cycles;
  // update the data in the moving window
  for(int j=0; j<cycles-1; j++) {
    Aver_ADC_value_filtered[channel][j] = Aver_ADC_value_filtered[channel][j+1];
  }
  Aver_ADC_value_filtered[channel][cycles-1] = interValue;
  // store this time's results for next calculation
  Aver_ADC_value_Prev[channel] = Aver_ADC_value[channel];
}

/**
 * Exponential moving average filter for the ADC value
 * @param int - channel: 0~ENABLED_CH-1
 * @param float - alpha: weighting decreasing coefficient (0~1)
 */
void ExponentialMovingFilter(int channel, float alpha) {
  if(alpha > 1) {
    alpha = 1;
  }
  else if(alpha < 0) {
    alpha = 0;
  } 
  Aver_ADC_value[channel] = alpha*Aver_ADC_value[channel]+(1-alpha)*Aver_ADC_value_Prev[channel];
  Aver_ADC_value_Prev[channel] = Aver_ADC_value[channel];
}

/**
 * Get the ADC value of all channels once
 */
void getADC(void) {
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

/**
 * Get average value from ADC for cycles
 * @param int - times: the cycles ADC go through before give you the detected value
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
  for(int t=0;t<ENABLED_CH;t++) {
    tempADCvalue[t] = tempADCvalue[t]/times;
    Aver_ADC_value[t] = (double)(tempADCvalue[t]*attRatio[t]*2.5)/16777251;  //24 bits
  }
}
