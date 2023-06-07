#define  DEBUG_SERIAL 1

#ifdef  DEBUG_SERIAL
#define SERIAL_PRINT(X)     Serial.print(X);  Serial.flush();
#define SERIAL_PRINTLN(X)   Serial.println(X); Serial.flush();

#define SERIAL_PRINT2(X,Y)     Serial.print(X); Serial.print(Y); Serial.flush();
#define SERIAL_PRINTLN2(X,Y)   Serial.print(X); Serial.println(Y); Serial.flush();

#else
#define SERIAL_PRINT(X)
#define SERIAL_PRINTLN(X)

#define SERIAL_PRINT2(X,Y)
#define SERIAL_PRINTLN2(X,Y)

#endif
