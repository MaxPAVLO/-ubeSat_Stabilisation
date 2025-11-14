#include "PID.h"

PID::PID(float kp, float ki, float kd, float dt)
  : kp(kp), ki(ki), kd(kd), dt(dt),
    setpoint(0), integral(0), prevError(0),
    outMin(-255), outMax(255) {}

void PID::setTunings(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PID::setSetpoint(float sp) {
  setpoint = sp;
}

void PID::setOutputLimits(float min, float max) {
  outMin = min;
  outMax = max;
}

float PID::compute(float input) {
  // Ошибка
  float error = setpoint - input;

  // Интегральная часть
  integral += error * dt;

  // Дифференциальная часть
  float derivative = (error - prevError) / dt;

  // PID
  float output = kp * error + ki * integral + kd * derivative;

  // Ограничим диапазон
  output = constrain(output, outMin, outMax);

  // Сохраняем ошибку
  prevError = error;

  return output;
}

void PID::reset() {
  integral = 0;
  prevError = 0;
}
