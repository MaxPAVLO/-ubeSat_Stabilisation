#include "Motors.h"

Motor::Motor(int pwm, int in1, int in2)
  : pwmPin(pwm), in1Pin(in1), in2Pin(in2) {}

void Motor::begin() {
  pinMode(pwmPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  stop();
}

void Motor::setSpeed(int s) {
  s = constrain(s, -255, 255);

  if (s > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, s);
  } 
  else if (s < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -s);
  } 
  else {
    stop();
  }
}

void Motor::stop() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(pwmPin, 0);
}
