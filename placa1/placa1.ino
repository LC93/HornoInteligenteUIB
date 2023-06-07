#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>

#include <HIB.h>
#include <SO.h>
#include <timerConfig.h>

// -------- Nuestras --------
#include "temperature.h"

// ----------- DEBUG ---------------
#define  DEBUG_SERIAL 1
#include "serialDebug.h"
// ---------------------------------

#define PERIOD_CONTROL_TEMP_TASK 4;

HIB hib;
SO so;

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);

/****************************
  Declaration of semaphores
****************************/
Sem s_tempOven;
Sem s_tGrill;
Sem s_goalTemp;

/***************************
  Declaration of mailboxes
***************************/
MBox mb_tOven;
MBox mb_tempDataToSend;

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
const unsigned char maskGrillOn = 0x02;

// Flag and masks to send smoke or temperature
// to taskTxCan
Flag f_txCan;
const unsigned char maskTempSend = 0x01;
// TODO: CAMBIAR MÁSCARA A 2, EL 0X00 ES SOLO DE PRUEBA
const unsigned char maskSmokeSend = 0x00;

// Flag and mask for CAN ISR to tell taskRxCan
// that a new message has been received
Flag f_rxCan;
const unsigned char maskCan = 0x01;

/**********************************
  Declaration of global variables
**********************************/
volatile uint8_t tGrill = TGRILL_OFF;
volatile uint16_t sampledTempOven;
volatile uint16_t slopeTemp;
volatile uint16_t goalTemp = NO_TEMP_GOAL;

/*****************
  CAN RX ISR
*****************/
void ISR_CAN() {
  char auxSREG;

  auxSREG = SREG;
  so.setFlag(f_rxCan, maskCan);
  SREG = auxSREG;
}

/*****************
  ADC handling
*****************/
void adcHook(uint16_t newAdcValue) {
  slopeTemp = newAdcValue;
  so.setFlag(f_adc, maskAdcEvent);
}

/*****************
  Timer 5 handling
*****************/
void timer5Hook() {
  so.updateTime();
}

/************************
  CAN tasks
*************************/
void taskRxCan() {
  
}

void taskTxCan() {
  int8_t value = 10;
  unsigned char mask = (maskTempSend | maskSmokeSend);
  unsigned char flagValue;
  uint32_t tx_id;
  

  while (true) {
    so.waitFlag(f_txCan, mask);
    flagValue = so.readFlag(f_txCan);

    switch (flagValue) {
      case maskTempSend:
        tx_id = TX_ID_TEMP;
        so.clearFlag(f_txCan, maskTempSend);
        break;
      case maskSmokeSend:
        
        so.clearFlag(f_txCan, maskSmokeSend);
        break;
    }
    
    if (CAN.checkPendingTransmission() != CAN_TXPENDING) {
      CAN.sendMsgBufNonBlocking(TX_ID_TEMP, CAN_EXTID, sizeof(int), (INT8U *) value);
    }
  }
}

/*****************************
  Temperature-realted tasks
*****************************/
void taskSimTemp() {
  uint16_t tOven = 0;
  float mappedSlope;

  while (true) {
    SERIAL_PRINTLN("taskSimTemp()");
    so.waitFlag(f_adc, maskAdcEvent);
    so.clearFlag(f_adc, maskAdcEvent);

    mappedSlope = map(slopeTemp,
                      MINIMUM_ADC_VALUE,
                      MAXIMUM_ADC_VALUE,
                      MINIMUM_SLOPE_VALUE,
                      MAXIMUM_SLOPE_VALUE
                     );
    //Serial.print("Temperatura horno preCalc: "); Serial.println(tOven);

    so.waitSem(s_tGrill);
    if (tGrill == TGRILL_OFF)
      mappedSlope = -mappedSlope;
    so.signalSem(s_tGrill);

    tOven = mappedSlope + tOven; // Funcion que simula la temperatura.

    if (tOven < 0)  // Limita la temperatura mínima simulada.
      tOven = 0;
    if (tOven > TGRILL_ON) // Limita la temperatura máxima simulada.
      tOven = TGRILL_ON;

    //Serial.print("mapped Slope: "); Serial.println(mappedSlope);
    //Serial.print("tGrill: "); Serial.println(tGrill);
    //Serial.print("  Temperatura horno simulada: "); Serial.println(tOven); // DEBUG

    so.signalMBox(mb_tOven, (byte *) &tOven);
    hib.ledToggle(1); // DEBUG

  }
}

void taskSensorTemp() {
  uint16_t tOven;
  uint16_t *tOvenMessage;

  while (true) {
    SERIAL_PRINTLN("taskSensorTemp()");
    so.waitMBox(mb_tOven, (byte**) &tOvenMessage);

    tOven = *tOvenMessage;

    //Serial.print("Temperatura horno en sensor: "); Serial.println(tOven); // DEBUG

    so.waitSem(s_tempOven);

    sampledTempOven = tOven;

    so.signalSem(s_tempOven);

    hib.ledToggle(2); // DEBUG
  }
}

void taskControlTemp() {
  unsigned long nextActivationTick;
  uint16_t tOven;

  nextActivationTick = so.getTick();

  while (true) {
    SERIAL_PRINTLN("taskControlTemp()");
    so.waitSem(s_tempOven);
    tOven = sampledTempOven;
    so.signalSem(s_tempOven);

    // Esto es un if de prueba, pero la temperatura
    // de referencia vendrá dada por la consigna
    so.waitSem(s_tGrill);
    //Serial.print("Control ve grill: "); Serial.println(tGrill);
    SERIAL_PRINTLN2("Control ve grill: ", tGrill);
    if (tOven >= 50 && s_tGrill != TGRILL_OFF) {
      //Serial.println("Control apaga grill");
      so.setFlag(f_temp, maskGrillOff);
    } else if (tOven < 50 && s_tGrill != TGRILL_ON) {
      //Serial.println("Control enciende grill");
      so.setFlag(f_temp, maskGrillOn);
    }

    so.signalSem(s_tGrill);

    SERIAL_PRINTLN2("Temperatura horno desde control: ", tOven); // DEBUG
    hib.ledToggle(3); // DEBUG

    nextActivationTick = nextActivationTick + PERIOD_CONTROL_TEMP_TASK;
    so.delayUntilTick(nextActivationTick);
  }

}

void taskGrill() {
  unsigned char mask = (maskGrillOff | maskGrillOn);
  unsigned char flagValue;

  while (true) {
    SERIAL_PRINTLN("taskGrill()");
    so.waitFlag(f_temp, mask);
    flagValue = so.readFlag(f_temp);
    so.clearFlag(f_temp, mask);

    so.waitSem(s_tGrill);
    SERIAL_PRINTLN2("Grill se ve: ", tGrill);
    if (flagValue == maskGrillOff && tGrill != TGRILL_OFF) {
      SERIAL_PRINTLN("Grill se apaga"); // DEBUG
      tGrill = TGRILL_OFF;
    } else if (flagValue == maskGrillOn && tGrill != TGRILL_ON) {
      SERIAL_PRINTLN("Grill se enciende"); // DEBUG
      tGrill = TGRILL_ON;
    }
    so.signalSem(s_tGrill);

    hib.ledToggle(4); // DEBUG
  }
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

  while (CAN.begin(CAN_500KBPS, MODE_LOOPBACK, true, false) != CAN_OK) {
    Serial.println("CAN BUS shield initiating");
    delay(100);
  }

  Serial.println("CAN BUS initiated");

  attachInterrupt(0, ISR_CAN, FALLING);
}

void loop() {
  s_tempOven = so.defSem(1);
  s_goalTemp = so.defSem(1);
  s_tGrill = so.defSem(1);

  mb_tOven = so.defMBox();
  mb_tempDataToSend = so.defMBox();

  f_adc = so.defFlag();
  f_temp = so.defFlag();
  f_txCan = so.defFlag();

  hib.setUpTimer5(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, timer5Hook);
  hib.adcSetTimerDriven(TIMER_TICKS_FOR_500ms, TIMER_PSCALER_FOR_500ms, adcHook);

  so.defTask(taskControlTemp, 1);
  so.defTask(taskSimTemp, 2);
  so.defTask(taskSensorTemp, 3);
  so.defTask(taskGrill, 4);

  so.enterMultiTaskingEnvironment();
}
