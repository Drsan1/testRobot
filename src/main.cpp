#include "RoboBasement.hpp"
#include "SmartServoBus.hpp"
#include <chrono>
#include <iostream>
#include <thread>

using namespace lx16a;

static SmartServoBus servoBus;
Motor motor;

const int numSensors = 2;
uint32_t maxDistance = 200;

UltrasonicSensor ultrasonicSensors[numSensors] = {
    UltrasonicSensor(3, 2, maxDistance),
    UltrasonicSensor(4, 19, maxDistance),
};

void Collision() {
    int numMeasurements = 5;
    bool stopRequested = false; 

    while (true) {
        bool isAnySensorClose = false;

        for (int sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
            ultrasonicSensors[sensorIndex].measureMultipleDistances(numMeasurements, false);
            double averageDistance = ultrasonicSensors[sensorIndex].getAverageDistance();
            if (averageDistance < 200) { // in mm
                if (!stopRequested) {
                    motor.requestStop();
                    stopRequested = true;
                }
                isAnySensorClose = true;
                break;
            }
        }

        if (!isAnySensorClose && stopRequested) {
            motor.resetStopRequest();
            stopRequested = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
}

void setup() {
    motor.motorSetup();

    Motor::Parameters params;
    params.wheelRadius = 5;
    params.wheelDistance = 10;

    servoBus.begin(1, UART_NUM_2, GPIO_NUM_14);
    ////////////////////////////////////////
    std::thread collisionThread(Collision);
    collisionThread.detach();

    motor.setLeftMotorId(1);
    motor.setRightMotorId(2);

    motor.moveForward(100, 9999);

    servoBus.set(0, 180_deg);
}

void loop() {
}