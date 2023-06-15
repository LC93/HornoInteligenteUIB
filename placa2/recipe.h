#include <Arduino.h>

typedef struct {
  uint16_t temperature;
  uint8_t totalTime;
} Phase;

class Recipe {
  private:
    const char* recipe;
    uint8_t totalPhases;
    Phase* phases;
    uint8_t currentPhase = 0;

  public:
    Recipe(const char* recipeName, uint8_t totalPhases, Phase *phases);
    Phase* getPhase();
    void nextPhase();
    bool finishedPhases();
    char* getName();

};
