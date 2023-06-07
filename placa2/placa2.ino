#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>

#include <HIB.h>
#include <SO.h>
#include <timerConfig.h>

#define PERIOD_CONTROL_TASK 5

HIB hib;
SO so;

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);

/***************************
  Declaration of mailboxes
***************************/
MBox mb_recipe;

void ISR_CAN() {

}

void keypadHook (uint8_t newKey) {

}

/*****************
  Timer 5 handling
*****************/
void timer5Hook() {
  so.updateTime();
}

void taskKeypad() {

}

void taskRxCan() {

}

void taskTxCan() {

}

void taskControl() {
  unsigned long nextActivationTick;
  uint8_t recipe = 2;
  
  nextActivationTick = so.getTick();
  while (true) {
    so.signalMBox(mb_recipe, (byte*) &recipe);
    nextActivationTick = nextActivationTick + PERIOD_CONTROL_TASK;
    so.delayUntilTick(nextActivationTick);
  }
}

void taskLog() {

}

void taskBuzzer() {

}

void taskLed() {

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

  hib.setUpTimer5(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, timer5Hook);
  hib.keySetIntDriven(100, keypadHook);

  so.defTask(taskControl, 1);
  so.defTask(task7Seg, 2);

  so.enterMultiTaskingEnvironment();
}
