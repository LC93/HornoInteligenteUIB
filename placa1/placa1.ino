#include "HIB.h"
#include "SO.h"
#include "timerConfig.h"

#include "temperature.h"

HIB hib;
SO so;

/****************************
  Declaration of semaphores
****************************/
Sem s_tempOven;
Sem s_tGrill;

/***************************
  Declaration of mailboxes
***************************/
MBox mb_tOven;

/***************************
  Declaration of mailboxes
***************************/
// Flag and mask for ADCHook to awake taskSimTemp
Flag f_adc;
const unsigned char maskAdcEvent = 0x01;

// Flag and masks for taskControlTemp to be able
// to control taskGrill
Flag f_temp;
const unsigned char maskGrillOff = 0x01;
const unsigned char maskGrillLow = 0x02;
const unsigned char maskGrillMedium = 0x04;
const unsigned char maskGrillHigh = 0x08;


/**********************************
  Declaration of global variables
**********************************/
volatile uint8_t tGrill = 100;
volatile float sampledTempOven;
volatile uint16_t slopeTemp;

/*****************
  ADC handling
*****************/
void adcHook(uint16_t newAdcValue) {
  slopeTemp = newAdcValue;
  so.setFlag(f_adc, maskAdcEvent);
}

/*****************************
  Temperature-realted tasks
*****************************/
void taskSimTemp() {
  uint16_t tOven;
  long mappedSlope;

  while (1) {
    so.waitFlag(f_adc, maskAdcEvent);
    so.clearFlag(f_adc, maskAdcEvent);

    mappedSlope = map(
                    slopeTemp,
                    MINIMUM_ADC_VALUE,
                    MAXIMUM_ADC_VALUE,
                    MINIMUM_SLOPE_VALUE,
                    MAXIMUM_SLOPE_VALUE
                  );

    tOven = tGrill * mappedSlope;

    //Serial.print("Temperatura horno: "); Serial.println(tOven); // DEBUG
    //Serial.flush(); //DEBUG

    so.signalMBox(mb_tOven, (byte *) &tOven);

  }
}

void taskSensorTemp() {
  uint16_t tOven;
  uint16_t *tOvenMessage;

  while (1) {
    so.waitMBox(mb_tOven, (byte**) &tOvenMessage);
    

    tOven = *tOvenMessage;

    //Serial.print("Temperatura horno en sensor: "); Serial.println(tOven); // DEBUG
    //Serial.flush(); // DEBUG

    // hib.ledToggle(1); // DEBUG
  }
}

void taskControlTemp() {

}

void taskGrill() {

}

/************************
  Smoke-realted tasks
*************************/
void taskSimSmoke() {

}

void taskSensorSmoke() {

}

void taskControlSmoke() {

}

void taskVent() {

}

void setup() {
  Serial.begin(115200);
  hib.begin();
  so.begin();
}

void loop() {
  s_tempOven = so.defSem(1);
  s_tGrill = so.defSem(1);
  mb_tOven = so.defMBox();
  f_adc = so.defFlag();
  f_temp = so.defFlag();

  hib.adcSetTimerDriven(TIMER_TICKS_FOR_500ms, TIMER_PSCALER_FOR_500ms, adcHook);

  so.defTask(taskSimTemp, 1);
  so.defTask(taskSensorTemp, 2);

  so.enterMultiTaskingEnvironment();
}
