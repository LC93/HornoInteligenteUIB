#include "recipe.h"

Recipe::Recipe(const char* r, uint8_t tp, Phase* p) :
  recipe(r),
  totalPhases(tp) {
  this->phases = new Phase[tp];

  for (size_t i = 0; i < tp; i++) {
    this->phases[i] = p[i];
  }
}

Phase* Recipe::getPhase() {
  return &(this->phases[this->currentPhase]);
}

void Recipe::nextPhase() {
  this->currentPhase++;
}

bool Recipe::finishedPhases() {
  return (currentPhase + 1) > totalPhases;
}

char* Recipe::getName() {
  return this->recipe;
}
