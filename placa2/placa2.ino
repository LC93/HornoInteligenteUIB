#include <math.h>

#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>

#include <HIB.h>
#include <SO.h>
#include <timerConfig.h>

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

HIB hib;
SO so;

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);

/****************************
  Declaration of semaphores
****************************/
Sem s_keypad;

/***************************
  Declaration of mailboxes
***************************/
MBox mb_recipe;
// Yo le cambiaría el nombre a este
// MBox
MBox mb_state;

/***************************
  Declaration of flags
***************************/
Flag f_keypad;
const unsigned char maskKeypad = 0x01;

Flag f_alarm;
const unsigned char maskFoodDone = 0x01;
const unsigned char maskFire = 0x02;

/**********************************
  Declaration of global variables
**********************************/
volatile uint8_t newKey;
volatile uint8_t pressedKey = hib.NO_KEY;
volatile bool isKeyNew = false;
const String recipeString[] = {
  "Pollo al horno", "Pizza", "Lasaña", "Bizcocho"
};

void ISR_CAN() {

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

void taskKeypad() {
  while (true) {
    so.waitFlag(f_keypad, maskKeypad);
    so.clearFlag(f_keypad, maskKeypad);
    so.waitSem(s_keypad);
    pressedKey = newKey;
    so.signalSem(s_keypad);
  }
}

void taskRxCan() {

}

void taskTxCan() {

}

void taskControl() {
  unsigned long nextActivationTick;
  int8_t selectedRecipe = 1;
  uint8_t currentState = IDLE_STATE;
  uint8_t lastPressedKey = hib.NO_KEY;
  struct LcdInfo info;

  nextActivationTick = so.getTick();
  while (true) {
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
    }
    so.signalSem(s_keypad);

    so.signalMBox(mb_recipe, (byte*) &selectedRecipe);
    nextActivationTick = nextActivationTick + PERIOD_CONTROL_TASK;
    so.delayUntilTick(nextActivationTick);
  }
}



void taskLog() {

}

void taskBuzzer() {
  unsigned char mask = (maskFoodDone | maskFire);
  unsigned char flagValue;
  uint8_t foodDoneSoundTimes = 5;
  uint8_t fireSoundTimes = 20;
  uint8_t ticksPerSound = 3;

  while (true) {
    so.waitFlag(f_alarm, mask);
    flagValue = so.readFlag(f_alarm);
    so.clearFlag(f_alarm, mask);

    if (flagValue == maskFoodDone) {
      for (uint8_t i = 0; i < foodDoneSoundTimes; i++) {
        playNote(SOL, OCTAVE, 500);
        delay(500);
      }
    } else if (flagValue == maskFire) {
      for (uint8_t i = 0; i < fireSoundTimes; i++) {
        for (uint8_t j = 0; j < ticksPerSound; j++) {
          playNote(SOL, OCTAVE, 100);
          delay(100);
        }
        delay(200);
      }
    }
  }
}

void taskLed() {
  unsigned char mask = (maskFoodDone | maskFire);
  unsigned char flagValue;

  while (true) {
    so.waitFlag(f_alarm, mask);
    flagValue = so.readFlag(f_alarm);

    if (flagValue == maskFoodDone) {

    } else if (flagValue == maskFire) {

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

  mb_recipe = so.defMBox();

  f_keypad = so.defFlag();
  f_alarm = so.defFlag();

  hib.setUpTimer5(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, timer5Hook);
  hib.keySetIntDriven(100, keypadHook);

  //so.defTask(taskControl, 1);
  so.defTask(taskBuzzer, 2);
  so.defTask(taskLed, 3);
  //so.defTask(taskKeypad, 4);
  //so.defTask(task7Seg, 5);
  //so.defTask(taskLcd, 6);

  so.enterMultiTaskingEnvironment();
}
