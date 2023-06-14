#include "recipe.h"

Recipe::Recipe(const char* r, uint8_t tp, Phase p[]) :
  recipe(r),
  totalPhases(tp),
  phases(p) {
  for (size_t i = 0; i < sizeof(p) / sizeof(Phase); i++) {
    Serial.println("Temp: "); Serial.println(p[i].temperature);
    Serial.println("Time: "); Serial.println(p[i].totalTime);

  }

}

Phase* Recipe::getPhase() {
  Serial.print("Temp: "); Serial.println((this->phases + (this->currentPhase * sizeof(Phase)))->temperature);
  return this->phases + (this->currentPhase * sizeof(Phase));
}

void Recipe::nextPhase() {
  this->currentPhase++;
}

bool Recipe::finishedPhases() {
  return currentPhase > totalPhases;
}

char* Recipe::getName() {
  return this->recipe;
}
