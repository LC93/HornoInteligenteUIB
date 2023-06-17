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
#define PERIOD_CONTROL_SMOKE_TASK 6;
#define PERIOD_SIM_SMOKE_TASK 5;
#define PERIOD_LOOPBACK_CAN_TASK 150; // Para probar can

typedef struct  {
  uint32_t id;
  uint16_t data;
} TxData;

HIB hib;
SO so;

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);

/****************************
  Declaration of semaphores
****************************/
Sem s_tempOven;
Sem s_smokeOven;
Sem s_tGrill;
Sem s_goalTemp;
Sem s_vent;

/***************************
  Declaration of mailboxes
***************************/
MBox mb_tOven;
MBox mb_smoke;
MBox mb_tempDataToSend;
MBox mb_smokeDataToSend;


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

Flag f_vent;
const unsigned char maskVentOff = 0x01;
const unsigned char maskVentOn = 0x02;

// Flag and masks to send smoke or temperature
// to taskTxCan
Flag f_txCan;
const unsigned char maskTempSend = 0x01;
const unsigned char maskSmokeSend = 0x02;

// Flag and mask for CAN ISR to tell taskRxCan
// that a new message has been received
Flag f_rxCan;
const unsigned char maskCan = 0x01;

/**********************************
  Declaration of global variables
**********************************/
volatile uint8_t tGrill = TGRILL_OFF;
volatile bool sVent = VENT_OFF;
volatile uint16_t sampledTempOven;
volatile float sampledSmokeOven;
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
  uint32_t rx_id;
  int rx_msg;

  while (true) {
    so.waitFlag(f_rxCan, maskCan);
    so.clearFlag(f_rxCan, maskCan);

    CAN.readRxMsg();

    rx_id = CAN.getRxMsgId();
    CAN.getRxMsgData((byte*) &rx_msg);

    so.waitSem(s_goalTemp);
    switch (rx_id) {
      case TEMP_GOAL_IDENTIFIER:
        goalTemp = rx_msg;
        break;
      case STOP_COOKING_IDENTIFIER:
        goalTemp = NO_TEMP_GOAL;
        break;
    }
    so.signalSem(s_goalTemp);

  }
}

void taskTxCan() {
  unsigned char mask = (maskTempSend | maskSmokeSend);
  unsigned char flagValue;
  TxData* dataToSendMsg;
  TxData dataToSend;

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
        so.waitMBox(mb_smokeDataToSend, (byte**) &dataToSendMsg);
        dataToSend = *dataToSendMsg;
        break;
    }

    if (CAN.checkPendingTransmission() != CAN_TXPENDING) {
      CAN.sendMsgBufNonBlocking(dataToSend.id, CAN_EXTID, sizeof(int), (INT8U *) &dataToSend.data);
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
  }
}

void taskSensorTemp() {
  uint16_t tOven;
  uint16_t *tOvenMessage;

  while (true) {
    so.waitMBox(mb_tOven, (byte**) &tOvenMessage);

    tOven = *tOvenMessage;

    so.waitSem(s_tempOven);

    sampledTempOven = tOven;

    so.signalSem(s_tempOven);
  }
}

void taskControlTemp() {
  unsigned long nextActivationTick;
  float maxHysteresis;
  float minHysteresis;
  bool reachedGoalTemp = false;
  int16_t lastGoalTemp = NO_TEMP_GOAL;
  TxData dataToSend;

  nextActivationTick = so.getTick();

  while (true) {
    // TODO: Esto es el logging, hay que probar
    // que funcione bien


    so.waitSem(s_goalTemp);
    if (goalTemp != NO_TEMP_GOAL) {
      so.waitSem(s_tempOven);
      dataToSend.id = TEMP_INFO_IDENTIFIER;
      dataToSend.data = sampledTempOven;
      so.signalMBox(mb_tempDataToSend, (byte *) &dataToSend);
      so.setFlag(f_txCan, maskTempSend);
      so.signalSem(s_tempOven);
      
      maxHysteresis = (float) goalTemp + goalTemp * HYSTERESIS_PERCENTAGE;
      minHysteresis = (float) goalTemp - goalTemp * HYSTERESIS_PERCENTAGE;

      if (goalTemp != lastGoalTemp) {
        lastGoalTemp = goalTemp;
        reachedGoalTemp = false;
      }

      so.signalSem(s_goalTemp);

      so.waitSem(s_tempOven);
      so.waitSem(s_tGrill);

      if ((sampledTempOven >= maxHysteresis) && (tGrill == TGRILL_ON)) {
        so.setFlag(f_temp, maskGrillOff);
      } else if ((sampledTempOven <= minHysteresis) && (tGrill == TGRILL_OFF)) {
        so.setFlag(f_temp, maskGrillOn);
      } else if (sampledTempOven < maxHysteresis
                 && sampledTempOven > minHysteresis
                 && !reachedGoalTemp) {
        reachedGoalTemp = true;

        dataToSend.id = GOAL_TEMPERATURE_REACHED_IDENTIFIER;
        so.signalMBox(mb_tempDataToSend, (byte *) &dataToSend);
        so.setFlag(f_txCan, maskTempSend);
      }

      so.signalSem(s_tGrill);
      so.signalSem(s_tempOven);
    } else {
      so.signalSem(s_goalTemp);

      // Si no hay consigna, hay que asegurarse de
      // que el grill quede apagado
      so.waitSem(s_tGrill);
      if (tGrill == TGRILL_ON)
        so.setFlag(f_temp, maskGrillOff);
      so.signalSem(s_tGrill);

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
  unsigned long nextActivationTick;
  uint16_t ldrAdcValue;
  float smokePercentage;

  nextActivationTick = so.getTick();

  while (true) {
    so.waitSem(s_vent);

    // Si el ventilador está apagado, obtendremos
    // el valor del LDR. En caso contrario, se irá
    // restando VENT_ON_SUBSTRACTION del valor actual
    // de smokePercentage hasta que llegue a 0
    if (!sVent) {
      ldrAdcValue = hib.ldrReadAdc(hib.RIGHT_LDR_SENS);

      smokePercentage = 1.0 - (map(ldrAdcValue,
                                   MINIMUM_ADC_VALUE,
                                   MAXIMUM_ADC_VALUE,
                                   MINIMUM_SMOKE_PERCENTAGE,
                                   MAXIMUM_SMOKE_PERCENTAGE
                                  ) / 100.0);
    } else {
      if ((smokePercentage - VENT_ON_SUBSTRACTION) < 0.0) {
        smokePercentage = 0.0;
      } else {
        smokePercentage -= VENT_ON_SUBSTRACTION;
      }
    }
    so.signalSem(s_vent);

    so.signalMBox(mb_smoke, (byte*) &smokePercentage);

    nextActivationTick = nextActivationTick + PERIOD_SIM_SMOKE_TASK;
    so.delayUntilTick(nextActivationTick);
  }
}

void taskSensorSmoke() {
  float* smokePercentageMsg;
  float smokePercentage;

  while (true) {
    so.waitMBox(mb_smoke, (byte**) &smokePercentageMsg);
    smokePercentage = *smokePercentageMsg;

    so.waitSem(s_smokeOven);
    sampledSmokeOven = smokePercentage;
    so.signalSem(s_smokeOven);
  }
}

void taskControlSmoke() {
  unsigned long nextActivationTick;
  TxData dataToSend;

  nextActivationTick = so.getTick();

  while (true) {
    so.waitSem(s_vent);
    so.waitSem(s_smokeOven);

    if (sampledSmokeOven >= MAX_ALLOWED_SMOKE && !sVent) {
      so.setFlag(f_vent, maskVentOn);

      dataToSend.id = FIRE_IDENTIFIER;
      so.signalMBox(mb_smokeDataToSend, (byte *) &dataToSend);
      so.setFlag(f_txCan, maskSmokeSend);
    } else if (sampledSmokeOven <= MIN_ALLOWED_SMOKE && sVent) {
      so.setFlag(f_vent, maskVentOff);
    }

    so.signalSem(s_smokeOven);
    so.signalSem(s_vent);

    nextActivationTick = nextActivationTick + PERIOD_CONTROL_SMOKE_TASK;
    so.delayUntilTick(nextActivationTick);
  }
}

void taskVent() {
  unsigned char mask = (maskVentOff | maskVentOn);
  unsigned char flagValue;

  while (true) {
    so.waitFlag(f_vent, mask);
    flagValue = so.readFlag(f_vent);
    so.clearFlag(f_vent, mask);

    so.waitSem(s_vent);
    if (flagValue == maskVentOff && sVent != VENT_OFF) {
      sVent = VENT_OFF;
    } else if (flagValue == maskVentOn && sVent != VENT_ON) {
      sVent = VENT_ON;
    }
    so.signalSem(s_vent);
  }

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
  s_smokeOven = so.defSem(1);
  s_goalTemp = so.defSem(1);
  s_tGrill = so.defSem(1);
  s_vent = so.defSem(1);

  mb_tOven = so.defMBox();
  mb_tempDataToSend = so.defMBox();
  mb_smokeDataToSend = so.defMBox();
  mb_smoke = so.defMBox();

  f_adc = so.defFlag();
  f_temp = so.defFlag();
  f_txCan = so.defFlag();
  f_rxCan = so.defFlag();
  f_vent = so.defFlag();

  hib.setUpTimer5(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, timer5Hook);
  hib.adcSetTimerDriven(TIMER_TICKS_FOR_500ms, TIMER_PSCALER_FOR_500ms, adcHook);

  so.defTask(taskControlTemp, 1);
  so.defTask(taskControlSmoke, 1);
  so.defTask(taskSimTemp, 2);
  so.defTask(taskSimSmoke, 2);
  so.defTask(taskSensorTemp, 3);
  so.defTask(taskSensorSmoke, 3);
  so.defTask(taskGrill, 4);
  so.defTask(taskVent, 4);
  so.defTask(taskRxCan, 5);
  so.defTask(taskTxCan, 6);



  so.enterMultiTaskingEnvironment();
}
