#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
private:
  float kp, ki, kd;     // коэффициенты
  float setpoint;       // целевое значение
  float integral;       // интегральная часть
  float prevError;      // предыдущая ошибка
  float outMin, outMax; // ограничения выхода
  float dt;             // шаг времени (в секундах)

public:
  PID(float kp, float ki, float kd, float dt);

  void setTunings(float kp, float ki, float kd);
  void setSetpoint(float sp);
  void setOutputLimits(float min, float max);

  float compute(float input); // вычислить новое значение
  void reset();               // сбросить внутренние состояния
};

#endif
