#include <Wire.h>
#include <MPU6050_light.h>
#include <MadgwickAHRS.h>
#include "Motors.h"
#include "PID.h"

// --- MPU и фильтр ---
MPU6050 mpu(Wire);
Madgwick filter;
const float FREQ_HZ = 100.0;
const unsigned long microsPerReading = 1000000.0 / FREQ_HZ;
unsigned long microsPrevious = 0;

// --- Пины TB6612FNG ---
const int AIN1 = 9, AIN2 = 10, PWMA = 11;
const int BIN1 = 7, BIN2 = 6, PWMB = 5, STBY = 8;

// --- Создаём два мотора ---
Motor motorA(PWMA, AIN1, AIN2);
Motor motorB(PWMB, BIN1, BIN2);

// --- PID контроллеры для X и Y ---
PID pidRoll(300, 1.0, 3.0, 0.01);  // kp, ki, kd, dt
PID pidPitch(300, 1.0, 3.0, 0.01);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  motorA.begin();
  motorB.begin();
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // включаем драйвер

  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("Ошибка MPU6050!");
    while(true) delay(1000);
  }
  mpu.calcOffsets();
  filter.begin(FREQ_HZ);

  pidRoll.setSetpoint(0); // хотим держать X = 0
  pidPitch.setSetpoint(0); // хотим держать Y = 0
  pidRoll.setOutputLimits(-255, 255);
  pidPitch.setOutputLimits(-255, 255);

  Serial.println("Система готова.");
}

void loop() {
  unsigned long microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    microsPrevious += microsPerReading;

    mpu.update();

    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float gx = mpu.getGyroX() * DEG_TO_RAD;
    float gy = mpu.getGyroY() * DEG_TO_RAD;
    float gz = mpu.getGyroZ() * DEG_TO_RAD;

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    float q0, q1, q2, q3;
    filter.getQuaternion(q0, q1, q2, q3);

    float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    // направление "верха" устройства
    float roll = 2.0f * (q1*q3 - q0*q2);
    float pitch = 2.0f * (q2*q3 + q0*q1);

    // --- PID вычисляем скорость моторов ---
    float pwmRoll = pidRoll.compute(roll);
    float pwmPitch = pidPitch.compute(pitch);

    // --- задаём скорость моторам ---
    motorA.setSpeed(pwmPitch);
    motorB.setSpeed(pwmRoll);

    // --- отладка ---
    Serial.print("Roll: "); Serial.print(roll, 3);
    Serial.print("\t Pitch: "); Serial.print(pitch, 3);
    Serial.print("\t pwmRoll: "); Serial.print(pwmRoll);
    Serial.print("\t pwmPitch: "); Serial.println(pwmPitch);

    delay(10);
  }
}
