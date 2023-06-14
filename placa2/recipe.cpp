#include "recipe.h"

Recipe::Recipe(const char* r, uint8_t tp, Phase p[]) : 
    recipe(r),
    totalPhases(tp),
    phases(p) {}

Phase* Recipe::getPhase() {
    return (Phase*) &(this->phases[this->currentPhase]);
}

void Recipe::nextPhase() {
    this->currentPhase++;
}

bool Recipe::finishedPhases() {
    return currentPhase > totalPhases;
}
