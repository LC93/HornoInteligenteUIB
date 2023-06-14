typedef struct {
  uint16_t temperature;
  uint16_t elapsedTime;
} SystemState;

enum LcdInfoType {
  SYSTEM_STATE,
  RECIPE
};

typedef struct {
  LcdInfoType type;
  char* recipe;
  SystemState state;
} LcdInfo;

enum LogType {
  FAILURE,
  INFO
};

typedef struct {
  LogType type;
  char* msg;
} LogInfo;
