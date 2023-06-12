#include <math.h>

#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>

#include <HIB.h>
#include <SO.h>
#include <timerConfig.h>

#include "C:\\Users\\mirp2\\Documents\\Arduino\\encastats\\practica-final\\canIdentifiers.h"

#define PERIOD_CONTROL_TASK 5

#define IDLE_STATE 0
#define COOKING_STATE 1

#define NUMBER_OF_RECIPES 4

#define KEY_1 0
#define KEY_2 1
#define KEY_3 2
#define KEY_4 3
#define KEY_5 4
#define KEY_6 5
#define KEY_7 6
#define KEY_8 7
#define KEY_9 8
#define KEY_STAR 9
#define KEY_0 10
#define KEY_HASHTAG 11

#define OCTAVE 7
#define SOL 8

struct LcdInfo {
  uint8_t line;
  char* data;
};

struct TxData {
  uint32_t id;
  uint16_t* data;
};

HIB hib;
SO so;

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);

/****************************
  Declaration of semaphores
****************************/
Sem s_keypad;
Sem s_fire;

/***************************
  Declaration of mailboxes
***************************/
MBox mb_recipe;
// Yo le cambiaría el nombre a este
// MBox
MBox mb_state;
MBox mb_txCan;

/***************************
  Declaration of flags
***************************/
Flag f_keypad;
const unsigned char maskKeypad = 0x01;

Flag f_alarm;
const unsigned char maskFoodDoneBuzzer = 0x01;
const unsigned char maskFoodDoneLed = 0x02;
const unsigned char maskFireBuzzer = 0x04;
const unsigned char maskFireLed = 0x08;

Flag f_rxCan;
const unsigned char maskCan = 0x01;

/**********************************
  Declaration of global variables
**********************************/
volatile uint8_t newKey;
volatile uint8_t pressedKey = hib.NO_KEY;
volatile bool isKeyNew = false;
//const String recipeString[] = {
//  "Pollo al horno", "Pizza", "Lasaña", "Bizcocho"
//};
volatile bool fire = true;


void ISR_CAN() {
  char auxSREG;

  auxSREG = SREG;
  so.setFlag(f_rxCan, maskCan);
  SREG = auxSREG;
}

void keypadHook (uint8_t pressedKey) {
  newKey = pressedKey;
  isKeyNew = true;
  so.setFlag(f_keypad, maskKeypad);
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

    switch (rx_id) {
      case FIRE_IDENTIFIER:
        so.waitSem(s_fire);
        fire = true;
        so.signalSem(s_fire);
        break;
      case TEMP_INFO_IDENTIFIER:
        break;
      case GOAL_TEMPERATURE_REACHED_IDENTIFIER:
        break;
    }
  }
}

void taskTxCan() {
  struct TxData* dataToSendMsg;
  struct TxData dataToSend;

  while (true) {
    so.waitMBox(mb_txCan, (byte**) &dataToSendMsg);
    dataToSend = *dataToSendMsg;

    if (CAN.checkPendingTransmission() != CAN_TXPENDING) {
      CAN.sendMsgBufNonBlocking(dataToSend.id, CAN_EXTID, sizeof(int), (INT8U *) dataToSend.data);
    }
  }
}

void taskKeypad() {
  while (true) {
    so.waitFlag(f_keypad, maskKeypad);
    so.clearFlag(f_keypad, maskKeypad);
    so.waitSem(s_keypad);
    pressedKey = newKey;
    so.signalSem(s_keypad);
  }
}

void taskControl() {
  unsigned long nextActivationTick;
  int8_t selectedRecipe = 1;
  uint8_t currentState = COOKING_STATE;
  uint8_t lastPressedKey = hib.NO_KEY;
  struct LcdInfo info;
  struct TxData* dataToSend = (TxData*) malloc(sizeof(TxData));

  nextActivationTick = so.getTick();
  while (true) {
    hib.ledToggle(0); // DEBUG
    if (currentState == IDLE_STATE) {
      so.waitSem(s_keypad);
      if (isKeyNew) {
        isKeyNew = false;
        // TODO: Hay un bug en el que si
        // se alterna entre el 2 y el 8 va
        // pegando botes por alguna razón
        if (pressedKey == KEY_2) {
          selectedRecipe = (selectedRecipe - 1) < 0 ?
                           NUMBER_OF_RECIPES - 1 :
                           (selectedRecipe - 1) % NUMBER_OF_RECIPES;
          Serial.print("Recipe: "); Serial.println(selectedRecipe);
        } else if (pressedKey == KEY_8) {
          selectedRecipe = (selectedRecipe + 1) % NUMBER_OF_RECIPES;
          Serial.print("Recipe: "); Serial.println(selectedRecipe);
        }
      }
      so.waitSem(s_keypad);
    } else if (currentState == COOKING_STATE) {
      so.waitSem(s_fire);
      if (fire) {
        dataToSend->id = STOP_COOKING_IDENTIFIER;
        so.signalMBox(mb_txCan, (byte*) dataToSend);
        so.setFlag(f_alarm, maskFireLed);
        so.setFlag(f_alarm, maskFireBuzzer);
      } else {

      }
      so.signalSem(s_fire);
    }

    nextActivationTick = nextActivationTick + PERIOD_CONTROL_TASK;
    so.delayUntilTick(nextActivationTick);
  }
}

void taskLog() {

}

void taskBuzzer() {
  unsigned char mask = (maskFoodDoneBuzzer | maskFireBuzzer);
  unsigned char flagValue;
  uint8_t foodDoneSoundTimes = 5;
  uint8_t fireSoundTimes = 20;
  uint8_t ticksPerSound = 3;

  while (true) {

    so.waitFlag(f_alarm, mask);
    hib.ledToggle(1);

    flagValue = so.readFlag(f_alarm);
    Serial.println(flagValue);

    if (flagValue == maskFoodDoneBuzzer) {
      so.clearFlag(f_alarm, maskFoodDoneBuzzer);
      for (uint8_t i = 0; i < foodDoneSoundTimes; i++) {
        playNote(SOL, OCTAVE, 500);
        delay(500);
      }
    } else if (flagValue == maskFireBuzzer) {
      so.clearFlag(f_alarm, maskFireBuzzer);
      Serial.println("There is a fire");
      for (uint8_t i = 0; i < fireSoundTimes; i++) {
        for (uint8_t j = 0; j < ticksPerSound; j++) {
          playNote(SOL, OCTAVE, 100);
          delay(100);
        }
        delay(200);
      }
      so.waitSem(s_fire);
      fire = false; // DEBUG
      so.signalSem(s_fire);
    }
  }
}

void taskLed() {
  unsigned char mask = (maskFoodDoneLed | maskFireLed);
  unsigned char flagValue;
  uint8_t foodDoneBlinks = 20;

  while (true) {
    so.waitFlag(f_alarm, mask);
    flagValue = so.readFlag(f_alarm);

    if (flagValue == maskFoodDoneLed) {
      so.clearFlag(f_alarm, maskFoodDoneLed);

    } else if (flagValue == maskFireLed) {
      so.clearFlag(f_alarm, maskFireLed);
      for (int i = 0; i < foodDoneBlinks; i++) {
        hib.ledToggle(3);
        hib.ledToggle(4);
        hib.ledToggle(5);
        delay(200);
      }
    }
  }
}

void taskLcd() {
  struct LcdInfo info;
  char buff[20];

  hib.lcdClear();
  hib.lcdPrint("Bienvenido/a!");

  while (true) {
    so.waitMBox(mb_state, (byte**) &info);
    hib.lcdClear();

    switch (info.line) {
      case 1:
        hib.lcdMoveHome();
        break;
      case 2:
        hib.lcdSetCursorSecondLine();
        break;
    }

    sprintf(buff, "%s", info.data);

    hib.lcdPrint(buff);
  }
}

void task7Seg() {
  uint8_t *recipeMsg;
  uint8_t recipe;

  while (true) {
    so.waitMBox(mb_recipe, (byte**) &recipeMsg);
    recipe = *recipeMsg;

    hib.d7sPrintDigit(recipe, hib.RIGHT_7SEG_DIS);
  }
}

/*****************
  Auxiliar functions
*****************/
void playNote(uint8_t note, uint8_t octave, uint16_t duration) {
  float calc = (((float) note) - 10.0) / 12.0 + ((float) octave) - 4.0;
  float frec = 440.0 * pow(2.0, calc);

  hib.buzzPlay(duration, frec);
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
  s_keypad = so.defSem(1);
  s_fire = so.defSem(1);

  mb_recipe = so.defMBox();
  mb_txCan = so.defMBox();

  f_keypad = so.defFlag();
  f_alarm = so.defFlag();
  f_rxCan = so.defFlag();

  hib.setUpTimer5(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, timer5Hook);
  hib.keySetIntDriven(100, keypadHook);

  so.defTask(taskControl, 1);
  so.defTask(taskLed, 2);
  so.defTask(taskBuzzer, 2);
  //so.defTask(taskKeypad, 4);
  //so.defTask(task7Seg, 5);
  //so.defTask(taskLcd, 6);

  so.enterMultiTaskingEnvironment();
}
