#include <math.h>

#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>

#include <HIB.h>
#include <SO.h>
#include <timerConfig.h>
#include <Terminal.h>

#include "C:\\Users\\mirp2\\Documents\\Arduino\\encastats\\practica-final\\canIdentifiers.h"
#include "recipe.h"
#include "lcd_and_logging.h"

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

typedef struct {
  uint32_t id;
  uint16_t data;
} TxData;

HIB hib;
SO so;
Terminal term;

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);

/****************************
  Declaration of semaphores
****************************/
Sem s_keypad;
Sem s_fire;
Sem s_currentTemp;
Sem s_reachedGoalTemp;

/***************************
  Declaration of mailboxes
***************************/
MBox mb_recipe;
MBox mb_lcd;
MBox mb_txCan;
MBox mb_log;

/***************************
  Declaration of flags
***************************/
Flag f_keypad;
const unsigned char maskKeypad = 0x01;

Flag f_alarm;
const unsigned char maskFoodDone = 0x01;
const unsigned char maskFire = 0x02;

Flag f_rxCan;
const unsigned char maskCan = 0x01;

/**********************************
  Declaration of global variables
**********************************/
volatile uint8_t newKey;
volatile uint8_t pressedKey = hib.NO_KEY;
volatile bool isKeyNew = false;
volatile bool fire = false;
volatile bool reachedGoalTemp = false;
volatile uint16_t currentTemp = 0;

Recipe* chicken;
Recipe* pizza;
Recipe* lasagna;
Recipe* cake;

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

    hib.ledToggle(0);
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
        so.waitSem(s_currentTemp);
        currentTemp = rx_msg;
        so.signalSem(s_currentTemp);
        break;
      case GOAL_TEMPERATURE_REACHED_IDENTIFIER:
        so.waitSem(s_reachedGoalTemp);
        reachedGoalTemp = true;
        so.signalSem(s_reachedGoalTemp);
        break;

        // DEBUG
        //      case TEMP_GOAL_IDENTIFIER:
        //        so.waitSem(s_reachedGoalTemp);
        //        reachedGoalTemp = true;
        //        so.signalSem(s_reachedGoalTemp);
        //        break;
    }
  }
}

void taskTxCan() {
  TxData* dataToSendMsg;
  TxData dataToSend;

  while (true) {
    so.waitMBox(mb_txCan, (byte**) &dataToSendMsg);
    dataToSend = *dataToSendMsg;

    Serial.println("Sending msg");

    if (CAN.checkPendingTransmission() != CAN_TXPENDING) {
      
      CAN.sendMsgBufNonBlocking(dataToSend.id, CAN_EXTID, sizeof(int), (INT8U *) &dataToSend.data);
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
  unsigned long initialPhaseTime;
  unsigned long currentPhaseTime = 0;
  unsigned long elapsedTime;
  unsigned long totalTime;
  int8_t selectedRecipeIndex = 0;
  uint8_t currentState = IDLE_STATE;
  uint8_t lastPressedKey = hib.NO_KEY;
  bool finishedCooking = false;
  bool inPhase = false;
  bool finishedPhase = false;
  char buff[50];
  LogInfo logInfo;
  LcdInfo lcdInfo;
  SystemState state;
  TxData dataToSend;
  Recipe* recipes[] = { chicken, pizza, lasagna, cake };
  Recipe* selectedRecipe;
  Phase* currentPhase;

  so.signalMBox(mb_recipe, (byte*) &selectedRecipeIndex);

  createLcdInfo(&lcdInfo,
                LcdInfoType::RECIPE,
                recipes[selectedRecipeIndex]->getName(),
                0,
                0);
  so.signalMBox(mb_lcd, (byte*) &lcdInfo);

  nextActivationTick = so.getTick();
  so.signalMBox(mb_lcd, (byte*) &lcdInfo);

  while (true) {
    if (currentState == IDLE_STATE) {
      lcdInfo.type = LcdInfoType::RECIPE;

      createLog(&logInfo, LogType::INFO, "Oven in idle state");
      so.signalMBox(mb_log, (byte*) &logInfo);

      so.waitSem(s_keypad);
      if (isKeyNew) {
        isKeyNew = false;

        if (pressedKey == KEY_2) {
          selectedRecipeIndex = (selectedRecipeIndex - 1) < 0 ?
                                NUMBER_OF_RECIPES - 1 :
                                (selectedRecipeIndex - 1) % NUMBER_OF_RECIPES;

          createLcdInfo(&lcdInfo,
                        LcdInfoType::RECIPE,
                        recipes[selectedRecipeIndex]->getName(),
                        0,
                        0);
          so.signalMBox(mb_lcd, (byte*) &lcdInfo);
        } else if (pressedKey == KEY_8) {
          selectedRecipeIndex = (selectedRecipeIndex + 1) % NUMBER_OF_RECIPES;
          createLcdInfo(&lcdInfo,
                        LcdInfoType::RECIPE,
                        recipes[selectedRecipeIndex]->getName(),
                        0,
                        0);
          so.signalMBox(mb_lcd, (byte*) &lcdInfo);
        } else if (pressedKey == KEY_5) {
          currentState = COOKING_STATE;

          selectedRecipe = recipes[selectedRecipeIndex];

          sprintf(buff, "User selected recipe %s", selectedRecipe->getName());
          createLog(&logInfo, LogType::INFO, buff);
          so.signalMBox(mb_log, (byte*) &logInfo);
        }
        so.signalMBox(mb_recipe, (byte*) &selectedRecipeIndex);
      }

      so.signalSem(s_keypad);
    } else if (currentState == COOKING_STATE) {
      sprintf(buff, "Oven currently cooking %s", selectedRecipe->getName());
      createLog(&logInfo, LogType::INFO, buff);
      so.signalMBox(mb_log, (byte*) &logInfo);

      so.waitSem(s_fire);
      if (fire) {
        so.signalSem(s_fire);

        dataToSend.id = STOP_COOKING_IDENTIFIER;
        so.signalMBox(mb_txCan, (byte*) &dataToSend);

        createLog(&logInfo, LogType::FAILURE, "There is a fire");
        so.signalMBox(mb_log, (byte*) &logInfo);

        so.setFlag(f_alarm, maskFire);

        currentState = IDLE_STATE;
        fire = false;
        finishedCooking = false;
        finishedPhase = false;
        inPhase = false;
        elapsedTime = 0;
      } else if (finishedCooking) {
        so.signalSem(s_fire);

        // Reset a todas las variables de control
        finishedCooking = false;
        finishedPhase = false;
        inPhase = false;
        elapsedTime = 0;

        currentState = IDLE_STATE;

        createLog(&logInfo, LogType::INFO, "Recipe finished!");
        so.signalMBox(mb_log, (byte*) &logInfo);

        so.setFlag(f_alarm, maskFoodDone);
      } else {
        so.signalSem(s_fire);

        so.waitSem(s_currentTemp);
        sprintf(buff, "Current temperature: %d", currentTemp);
        so.signalSem(s_currentTemp);

        createLog(&logInfo, LogType::INFO, buff);
        so.signalMBox(mb_log, (byte*) &logInfo);

        if (selectedRecipe->finishedPhases() && finishedPhase) {
          finishedCooking = true;
          createCanMsg(&dataToSend, STOP_COOKING_IDENTIFIER, 0);
          so.signalMBox(mb_txCan, (byte*) &dataToSend);

          createLog(&logInfo, LogType::INFO, "Finished");
          so.signalMBox(mb_log, (byte*) &logInfo);
        } else if (!selectedRecipe->finishedPhases() && finishedPhase) {
          finishedPhase = false;
          inPhase = false;
          elapsedTime = 0;

          createLog(&logInfo, LogType::INFO, "Finished phase, but not recipe yet");
          so.signalMBox(mb_log, (byte*) &logInfo);
        } else if (!inPhase) {
          currentPhase = selectedRecipe->getPhase();
          inPhase = true;

          createCanMsg(&dataToSend, TEMP_GOAL_IDENTIFIER, currentPhase->temperature);
          so.signalMBox(mb_txCan, (byte*) &dataToSend);

          createLog(&logInfo, LogType::INFO, "Not into phase, sending temperature goal");
          so.signalMBox(mb_log, (byte*) &logInfo);
        } else {
          so.waitSem(s_reachedGoalTemp);
          if (reachedGoalTemp && (elapsedTime >= currentPhase->totalTime)) {
            currentPhaseTime = 0;
            finishedPhase = true;

            selectedRecipe->nextPhase();

            createLog(&logInfo, LogType::INFO, "Finished phase");
            so.signalMBox(mb_log, (byte*) &logInfo);
          } else if (reachedGoalTemp && (currentPhaseTime == 0)) {
            initialPhaseTime = millis();
            currentPhaseTime = millis();
            elapsedTime = (currentPhaseTime - initialPhaseTime) / 1000;

            createLog(&logInfo, LogType::INFO, "Initializing phase");
            so.signalMBox(mb_log, (byte*) &logInfo);
          } else if (reachedGoalTemp &&
                     currentPhaseTime != 0 &&
                     elapsedTime < currentPhase->totalTime) {
            currentPhaseTime = millis();
            totalTime++;
            elapsedTime = (currentPhaseTime - initialPhaseTime) / 1000;

            so.waitSem(s_currentTemp);
            createLcdInfo(&lcdInfo, LcdInfoType::SYSTEM_STATE, "", currentTemp, elapsedTime);
            so.signalSem(s_currentTemp);
            so.signalMBox(mb_lcd, (byte*) &lcdInfo);

            sprintf(buff, "Elapsed time: %d", elapsedTime);
            createLog(&logInfo, LogType::INFO, buff);
            so.signalMBox(mb_log, (byte*) &logInfo);
          }
          so.signalSem(s_reachedGoalTemp);
        }
      }
    }

    nextActivationTick = nextActivationTick + PERIOD_CONTROL_TASK;
    so.delayUntilTick(nextActivationTick);
  }
}

void taskLog() {
  LogInfo* logInfoMsg;
  LogInfo logInfo;

  while (true) {
    so.waitMBox(mb_log, (byte**) &logInfoMsg);
    logInfo = *logInfoMsg;

    switch (logInfo.type) {
      case LogType::FAILURE:
        term.print("[ERROR] ");
        term.println(logInfo.msg);
        break;
      case LogType::INFO:
        term.print("[INFO] ");
        term.println(logInfo.msg);
        break;
    }
  }
}

void taskAlarm() {
  unsigned char mask = (maskFoodDone | maskFire);
  unsigned char flagValue;
  uint8_t foodDoneBlinks = 20;
  uint8_t foodDoneSoundTimes = 5;
  uint8_t ticksPerSound = 1;

  while (true) {
    so.waitFlag(f_alarm, mask);
    flagValue = so.readFlag(f_alarm);
    so.clearFlag(f_alarm, mask);

    if (flagValue == maskFoodDone) {
      for (uint8_t i = 0; i < foodDoneSoundTimes; i++) {
        hib.ledToggle(0);
        hib.ledToggle(1);
        hib.ledToggle(2);
        hib.ledToggle(3);
        hib.ledToggle(4);
        hib.ledToggle(5);
        playNote(SOL, OCTAVE, 500);
        delay(500);
      }
    } else if (flagValue == maskFire) {
      for (int i = 0; i < foodDoneBlinks; i++) {
        hib.ledToggle(0);
        hib.ledToggle(1);
        hib.ledToggle(2);
        hib.ledToggle(3);
        hib.ledToggle(4);
        hib.ledToggle(5);
        for (uint8_t j = 0; j < ticksPerSound; j++) {
          playNote(SOL, OCTAVE, 100);
          delay(100);
        }
        delay(200);
      }
    }

    // Nos aseguramos de que los
    // leds quedan apagados
    hib.ledOff(0);
    hib.ledOff(1);
    hib.ledOff(2);
    hib.ledOff(3);
    hib.ledOff(4);
    hib.ledOff(5);
  }
}

void taskLcd() {
  LcdInfo* infoMsg;
  LcdInfo info;
  char buff[50];

  while (true) {
    so.waitMBox(mb_lcd, (byte**) &infoMsg);
    info = *infoMsg;
    hib.lcdClear();

    switch (info.type) {
      case LcdInfoType::SYSTEM_STATE:
        sprintf(buff,
                "Temp: %d",
                info.state.temperature);
        hib.lcdPrint(buff);
        hib.lcdMoveHome();
        hib.lcdSetCursorSecondLine();
        sprintf(buff,
                "Time: %d",
                info.state.elapsedTime);
        hib.lcdPrint(buff);
        break;
      case LcdInfoType::RECIPE:
        hib.lcdPrint(info.recipe);
        break;
    }
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

void createLog(LogInfo* info, LogType type, const char* msg) {
  char buff[50];
  info->type = type;
  sprintf(buff, "%s", msg);
  info->msg = buff;
}

void createCanMsg(TxData* data, uint32_t id, uint16_t msg) {
  data->id = id;
  data->data = msg;
}

void createLcdInfo(LcdInfo* info,
                   LcdInfoType type,
                   char* recipe,
                   uint16_t temperature,
                   uint16_t elapsedTime) {
  info->type = type;
  info->recipe = recipe;
  info->state.temperature = temperature;
  info->state.elapsedTime = elapsedTime;
}

void setup() {
  Serial.begin(115200);
  hib.begin();
  so.begin();
  term.begin(115200);

  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
    Serial.println("CAN BUS shield initiating");
    delay(100);
  }

  Serial.println("CAN BUS initiated");

  attachInterrupt(0, ISR_CAN, FALLING);

  Phase chickenPhases[] = {
    {200, 50},
    {150, 10}
  };
  chicken = new Recipe("Pollo al horno", 2, chickenPhases);

  Phase pizzaPhases[] = {
    {220, 20}
  };
  pizza = new Recipe("Pizza", 1, pizzaPhases);

  Phase lasagnaPhases[] = {
    {180, 20},
    {200, 8}
  };
  lasagna = new Recipe("Lasaña", 2, lasagnaPhases);

  Phase cakePhases[] = {
    {170, 10},
    {220, 20},
    {150, 10}
  };
  cake = new Recipe("Bizcocho", 3, cakePhases);
}

void loop() {
  s_keypad = so.defSem(1);
  s_fire = so.defSem(1);
  s_currentTemp = so.defSem(1);
  s_reachedGoalTemp = so.defSem(1);

  mb_lcd = so.defMBox();
  mb_recipe = so.defMBox();
  mb_txCan = so.defMBox();
  mb_log = so.defMBox();

  f_keypad = so.defFlag();
  f_alarm = so.defFlag();
  f_rxCan = so.defFlag();

  hib.setUpTimer5(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, timer5Hook);
  hib.keySetIntDriven(100, keypadHook);

  so.defTask(taskControl, 1);
  so.defTask(taskAlarm, 2);
  so.defTask(taskKeypad, 3);
  so.defTask(taskLog, 4);
  so.defTask(taskLcd, 5);
  so.defTask(task7Seg, 6);
  so.defTask(taskRxCan, 7);
  so.defTask(taskTxCan, 8);

  so.enterMultiTaskingEnvironment();
}
