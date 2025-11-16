#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

class Motor {
private:
  int pwmPin;
  int in1Pin;
  int in2Pin;

public:
  Motor(int pwm, int in1, int in2);

  void begin();         // Настройка пинов
  void setSpeed(int s); // Задать скорость (-255...255)
  void stop();          // Полная остановка
};

#endif
