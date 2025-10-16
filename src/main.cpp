#include <Arduino.h>
#include <Wire.h>
#include <SCMD.h>
#include <SCMD_config.h>
#include <PID_v1.h>

SCMD motor;

#define MOTOR 1

const int LDRPin = A3;
const int buttonPin = 12;
const int lightThreshold = 200;
const int slits = 4;

double motorPower = 100;
double targetSpeed = 10;
double speed = 0;
float minPower = 50;

double Kp = 15;
double Ki = 1;
double Kd = 0.3;

unsigned long currentTime;
unsigned long deltaTime;


PID pid(&speed, &motorPower, &targetSpeed, Kp, Ki, Kd, DIRECT);

bool lastState = 0;
unsigned long lastPulse = 0;


void setup() {
    Serial.begin(9600);
    Wire.begin();
    pinMode(buttonPin, INPUT_PULLDOWN);

    motor.settings.commInterface = I2C_MODE;
    motor.settings.I2CAddress = 0x5D;

    while (motor.begin() != 0xA9) {
        Serial.println("Motor ID mismatch, retrying...");
        delay(1);
    }
    Serial.println("Motor ID matched");

    while (!motor.ready()) {
      delay(1);
    }

    while (motor.busy()) {
      delay(1);
    }
    motor.enable();
    motor.setDrive(MOTOR, 1, motorPower);

    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255);
    pid.SetSampleTime(50);

    randomSeed(analogRead(A0));
}

void loop() {
  bool state = analogRead(LDRPin) > lightThreshold;
    if ((state) && !lastState) {
        currentTime = millis();
        if (lastPulse > 0) {
            deltaTime = currentTime - lastPulse;
            speed = (2.0 * 3.14159 / slits) / (deltaTime / 1000.0);
            pid.Compute();

            if (motorPower < minPower) motorPower = minPower;
            motor.setDrive(MOTOR, 1, motorPower);
        }
        lastPulse = currentTime;
    }
    lastState = state;

    if (digitalRead(buttonPin)) {
        targetSpeed = random(1, 61);
        delay(100);
    }

    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print(" | Target: ");
    Serial.println(targetSpeed);

    delay(10);
}
