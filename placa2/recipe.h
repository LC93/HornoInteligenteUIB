#include <Arduino.h>

typedef struct {
    uint8_t totalTime;
    uint16_t temperature;
} Phase;

class Recipe {
    private:
        const char* recipe;
        uint8_t totalPhases;
        Phase* phases;
        uint8_t currentPhase = 0;
    
    public:
        Recipe(const char* recipeName, uint8_t totalPhases, Phase phases[]);
        Phase* getPhase();
        void nextPhase();
        bool finishedPhases();
        char* getName();

};
