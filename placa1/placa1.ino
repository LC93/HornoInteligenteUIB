#include "HIB.h"
#include "SO.h"
#include "timerConfig.h"

HIB hib;
SO so;

#define MINIMUM_ADC_VALUE 0
#define MAXIMUM_ADC_VALUE 1023
#define MINIMUM_SLOPE_VALUE 50
#define MAXIMUM_SLOPE_VALUE 150

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
volatile uint8_t tGrill = 100;
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
  uint16_t tOven;
  long mappedTOven;

  while (1) {
    so.waitFlag(f_adc, maskAdcEvent);
    so.clearFlag(f_adc, maskAdcEvent);

    mappedTOven = map(
      slopeTemp, 
      MINIMUM_ADC_VALUE, 
      MAXIMUM_ADC_VALUE, 
      MINIMUM_SLOPE_VALUE, 
      MAXIMUM_SLOPE_VALUE
    );

    tOven = tGrill * slopeTemp;

    // Serial.print("Temperatura horno: "); Serial.println(tOven); // DEBUG
    // Serial.flush(); //DEBUG

    so.signalMBox(mb_tOven, (byte *) &tOven);

  }


}

void taskSensorTemp() {
  float tOven;
  float *tOvenMessage;
  
  so.waitMBox(mb_tOven, (byte**) &tOvenMessage);
  
  //Serial.print("Temperatura horno en sensor: "); Serial.println(*tOvenMessage); // DEBUG
  //Serial.flush(); // DEBUG

  tOven = *tOvenMessage;
  hib.ledToggle(1);

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

  hib.adcSetTimerDriven(TIMER_TICKS_FOR_500ms, TIMER_PSCALER_FOR_500ms, adcHook);
  so.defTask(taskSimTemp, 2);
  so.defTask(taskSensorTemp, 1);

  so.enterMultiTaskingEnvironment();
}
