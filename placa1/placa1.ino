#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>

#include <HIB.h>
#include <SO.h>
#include <timerConfig.h>

#include "temperature.h"
// A falta de mejor solución, hay que copiar
// el path completo al header porque Arduino no
// soporta includes relativos
#include "C:\\Users\\mirp2\\Documents\\Arduino\\encastats\\practica-final\\canIdentifiers.h"
#include "serialDebug.h"

#define PERIOD_CONTROL_TEMP_TASK 4;
#define PERIOD_LOOPBACK_CAN_TASK 150; // Para probar can

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
  Declaration of flags
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

struct TxData {
  uint32_t id;
  uint16_t* data;
};

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

void taskLoopbackCan() {
  unsigned long nextActivationTick; // Para probar con tx
  size_t i = 0;
  uint16_t tOvens[] = { 180, 200, 220, 150 };
  struct TxData data;

  data.id = TEMP_GOAL_IDENTIFIER;

  nextActivationTick = so.getTick(); // Para probar con tx

  while (true) {
    data.data = &tOvens[i];
    so.signalMBox(mb_tempDataToSend, (byte *) &data); // Para probar con tx
    so.setFlag(f_txCan, maskTempSend); // Para probar con tx

    i = (i + 1) % 4;

    nextActivationTick = nextActivationTick + PERIOD_LOOPBACK_CAN_TASK; // Para probar con tx
    so.delayUntilTick(nextActivationTick); // Para probar con tx
  }
}

/************************
  CAN tasks
*************************/
void taskRxCan() {
  uint32_t rx_id;
  int rx_msg;

  while (true) {
    so.waitFlag(f_rxCan, maskCan);
    so.clearFlag(f_rxCan, maskCan);

    CAN.readRxMsg();

    rx_id = CAN.getRxMsgId();
    CAN.getRxMsgData((byte*) &rx_msg);

    switch (rx_id) {
      case TEMP_GOAL_IDENTIFIER:
        so.waitSem(s_goalTemp);
        goalTemp = rx_msg;
        so.signalSem(s_goalTemp);
        break;
      case STOP_COOKING_IDENTIFIER:
        so.waitSem(s_goalTemp);
        goalTemp = NO_TEMP_GOAL;
        so.signalSem(s_goalTemp);
        break;
    }
  }
}

void taskTxCan() {
  unsigned char mask = (maskTempSend | maskSmokeSend);
  unsigned char flagValue;
  struct TxData* dataToSendMsg;
  struct TxData dataToSend;

  while (true) {
    so.waitFlag(f_txCan, mask);
    flagValue = so.readFlag(f_txCan);

    switch (flagValue) {
      case maskTempSend:
        so.clearFlag(f_txCan, maskTempSend);
        so.waitMBox(mb_tempDataToSend, (byte**) &dataToSendMsg);
        dataToSend = *dataToSendMsg;
        break;
      case maskSmokeSend:
        so.clearFlag(f_txCan, maskSmokeSend);
        // TODO: Same para el humo
        break;
    }

    if (CAN.checkPendingTransmission() != CAN_TXPENDING) {
      CAN.sendMsgBufNonBlocking(dataToSend.id, CAN_EXTID, sizeof(int), (INT8U *) dataToSend.data);
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
    so.waitFlag(f_adc, maskAdcEvent);
    so.clearFlag(f_adc, maskAdcEvent);

    mappedSlope = map(slopeTemp,
                      MINIMUM_ADC_VALUE,
                      MAXIMUM_ADC_VALUE,
                      MINIMUM_SLOPE_VALUE,
                      MAXIMUM_SLOPE_VALUE
                     );

    so.waitSem(s_tGrill);
    if (tGrill == TGRILL_OFF)
      mappedSlope = -mappedSlope;
    so.signalSem(s_tGrill);

    tOven = mappedSlope + tOven; // Funcion que simula la temperatura.

    if (tOven < 0)  // Limita la temperatura mínima simulada.
      tOven = 0;
    if (tOven > TGRILL_ON) // Limita la temperatura máxima simulada.
      tOven = TGRILL_ON;

    so.signalMBox(mb_tOven, (byte *) &tOven);
    // hib.ledToggle(1); // DEBUG

  }
}

void taskSensorTemp() {
  uint16_t tOven;
  uint16_t *tOvenMessage;

  while (true) {
    so.waitMBox(mb_tOven, (byte**) &tOvenMessage);

    tOven = *tOvenMessage;

    //Serial.print("Temperatura horno en sensor: "); Serial.println(tOven); // DEBUG

    so.waitSem(s_tempOven);

    sampledTempOven = tOven;

    so.signalSem(s_tempOven);

    // hib.ledToggle(2); // DEBUG
  }
}

void taskControlTemp() {
  unsigned long nextActivationTick;
  float maxHysteresis;
  float minHysteresis;
  bool reachedGoalTemp = false;
  int16_t lastGoalTemp = NO_TEMP_GOAL;
  struct TxData* dataToSend = (TxData*) malloc(sizeof(TxData));

  nextActivationTick = so.getTick();

  while (true) {
    hib.ledToggle(0);

    // TODO: Esto es el logging, hay que probar
    // que funcione bien
    so.waitSem(s_tempOven);
    dataToSend->id = TEMP_INFO_IDENTIFIER;
    dataToSend->data = &sampledTempOven;
    so.setFlag(f_txCan, maskTempSend);
    so.signalMBox(mb_tempDataToSend, (byte *) dataToSend);
    so.signalSem(s_tempOven);

    so.waitSem(s_goalTemp);
    if (goalTemp != NO_TEMP_GOAL) {
      maxHysteresis = (float) goalTemp + goalTemp * HYSTERESIS_PERCENTAGE;
      minHysteresis = (float) goalTemp - goalTemp * HYSTERESIS_PERCENTAGE;
      //SERIAL_PRINTLN2("Max hysteresis: ", maxHysteresis);
      //SERIAL_PRINTLN2("Min hysteresis: ", minHysteresis);

      if (goalTemp != lastGoalTemp) {
        lastGoalTemp = goalTemp;
        reachedGoalTemp = false;
      }

      so.signalSem(s_goalTemp);

      so.waitSem(s_tempOven);
      so.waitSem(s_tGrill);

      SERIAL_PRINTLN2("Current temp: ", sampledTempOven);

      if (sampledTempOven >= maxHysteresis && tGrill != TGRILL_OFF) {
        so.setFlag(f_temp, maskGrillOff);
      } else if (sampledTempOven <= minHysteresis && tGrill != TGRILL_ON) {
        so.setFlag(f_temp, maskGrillOn);
      } else if (sampledTempOven < maxHysteresis
                 && sampledTempOven > minHysteresis
                 && !reachedGoalTemp) {
        //SERIAL_PRINTLN("Goal temp reached!");
        reachedGoalTemp = true;
        dataToSend->id = GOAL_TEMPERATURE_REACHED_IDENTIFIER;
        so.setFlag(f_txCan, maskTempSend);
        so.signalMBox(mb_tempDataToSend, (byte *) dataToSend);
      }

      so.signalSem(s_tGrill);
      so.signalSem(s_tempOven);
    } else {
      so.waitSem(s_tGrill);
      // Si no hay consigna, hay que asegurarse de
      // que el grill quede apagado
      if (tGrill == TGRILL_ON)
        so.setFlag(f_temp, maskGrillOff);
      so.signalSem(s_tGrill);

      so.signalSem(s_goalTemp);
    }

    nextActivationTick = nextActivationTick + PERIOD_CONTROL_TEMP_TASK;
    so.delayUntilTick(nextActivationTick);
  }
}

void taskGrill() {
  unsigned char mask = (maskGrillOff | maskGrillOn);
  unsigned char flagValue;

  while (true) {
    so.waitFlag(f_temp, mask);
    flagValue = so.readFlag(f_temp);
    so.clearFlag(f_temp, mask);

    so.waitSem(s_tGrill);
    if (flagValue == maskGrillOff && tGrill != TGRILL_OFF) {
      tGrill = TGRILL_OFF;
    } else if (flagValue == maskGrillOn && tGrill != TGRILL_ON) {
      tGrill = TGRILL_ON;
    }
    so.signalSem(s_tGrill);

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

  // while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
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
  f_rxCan = so.defFlag();

  hib.setUpTimer5(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, timer5Hook);
  hib.adcSetTimerDriven(TIMER_TICKS_FOR_500ms, TIMER_PSCALER_FOR_500ms, adcHook);

  so.defTask(taskControlTemp, 1);
  so.defTask(taskSimTemp, 2);
  so.defTask(taskSensorTemp, 3);
  so.defTask(taskGrill, 4);
  so.defTask(taskLoopbackCan, 7);
  so.defTask(taskRxCan, 5);
  so.defTask(taskTxCan, 6);

  so.enterMultiTaskingEnvironment();
}
