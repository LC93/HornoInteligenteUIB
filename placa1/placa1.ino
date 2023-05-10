#include "HIB.h"
#include "SO.h"
#include "timerConfig.h"

HIB hib;
SO so;


/***************************
  Declaration of semaphores
***************************/
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

/**************************
  Declaration of global variables
***************************/
volatile uint8_t tGrill;
volatile float sampledTempOven;
volatile uint16_t slopeTemp;

/*****************
  ADC handling
******************/
void adcHook(uint16_t newAdcValue) {
  slopeTemp = newAdcValue;
  so.setFlag(f_adc, maskAdcEvent);

}

void taskGrill() {

}

void taskSimTemp() {
  while (1) {
    so.waitFlag(f_adc, maskAdcEvent);
    so.clearFlag(f_adc, maskAdcEvent);
    hib.ledToggle(1);
  }


}

void taskSensorTemp() {

}

void taskControlTemp() {

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

  hib.adcSetTimerDriven(TIMER_TICKS_FOR_250ms, TIMER_PSCALER_FOR_250ms, adcHook);
  so.defTask(taskSimTemp, 1);

  so.enterMultiTaskingEnvironment();
}
