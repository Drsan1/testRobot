#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "RBCX.h"
#include <Arduino.h>
#include <cmath>
#include <chrono>
#include <thread>

// The Motor class represents a motor with basic movement capabilities.
class Motor {
    std::atomic<bool> stopRequested{false}; 
public:
    // Structure to hold parameters
    struct Parameters {
        int wheelDistance; // in mm
        int wheelRadius; // in mm
        int encoderTicksPerRevolution = 2000;

        float mmToTicks = encoderTicksPerRevolution / (2 * M_PI * wheelRadius);
        float ticksToMm = 2.f * M_PI * wheelRadius / encoderTicksPerRevolution;
    };

    // Setter methods for motor IDs
    void setLeftMotorId(int id);
    void setRightMotorId(int id);

    // Basic movements
    // Initiates forward movement of the motor at a specified speed and for a given distance.
    // Optionally, the motor can be stopped automatically at the end of the movement.
    void moveForward(int speed, int distance, bool stopAtEnd = true);
    // Moves the motor backward at a given speed.
    void moveBackward(int speed, int distance, bool stopAtEnd);
    // Turns the motor left at a given speed.
    void turnLeft(int speed, int angle);
    // Turns the motor right at a given speed.
    void turnRight(int speed, int angle);
    // Makes the motor arc left at given inner and outer speeds.
    void arcLeft(int speed, int angle, int radius);
    // Makes the motor arc right at given inner and outer speeds.
    void arcRight(int speed, int angle, int radius);
    // Stops the motor.
    void stop();
    
    void requestStop() {
        stopRequested.store(true); // Set the stop flag
    }

    void resetStopRequest() {
        stopRequested.store(false); // Reset the stop flag at the start of a movement
    }

private:
    Parameters params;
    // Motor IDs for left and right motor.
    rb::MotorId leftMotorId;
    rb::MotorId rightMotorId;

    // Sets the speed of the motor with a given ID.
    void setMotorSpeed(int motorId, int speed, int durationMs);
};

// Implementation

auto& man = rb::Manager::get();

void Motor::setLeftMotorId(int id) {
    switch (id) {
    case 1:
        leftMotorId = rb::MotorId::M1;
        break;
    case 2:
        leftMotorId = rb::MotorId::M2;
        break;
    case 3:
        leftMotorId = rb::MotorId::M3;
        break;
    case 4:
        leftMotorId = rb::MotorId::M4;
        break;
    default:
        throw std::invalid_argument("Invalid motor ID");
    }
}

void Motor::setRightMotorId(int id) {
    switch (id) {
    case 1:
        rightMotorId = rb::MotorId::M1;
        break;
    case 2:
        rightMotorId = rb::MotorId::M2;
        break;
    case 3:
        rightMotorId = rb::MotorId::M3;
        break;
    case 4:
        rightMotorId = rb::MotorId::M4;
        break;
    default:
        throw std::invalid_argument("Invalid motor ID");
    }
}

// Basic movements
void Motor::moveForward(int speed, int distance, bool stopAtEnd) {
    resetStopRequest();
    man.motor(leftMotorId).setCurrentPosition(0);
    man.motor(rightMotorId).setCurrentPosition(0);

    int ticks_M1 = 0;
    distance = distance / params.mmToTicks;

    while (ticks_M1 < distance) {
        while (stopRequested.load()) { 
            delay(10);
        }
        man.motor(leftMotorId).speed(-speed);
        man.motor(rightMotorId).speed(speed);

        man.motor(leftMotorId).requestInfo([&ticks_M1](rb::Motor& info) {
            ticks_M1 = -info.position();
        });

        delay(10);
    }

    if (stopAtEnd) {
        // Stop motors at the end
        man.motor(leftMotorId).speed(0);
        man.motor(rightMotorId).speed(0);
    }
}

void Motor::moveBackward(int speed, int distance, bool stopAtEnd) {
    resetStopRequest();
    man.motor(leftMotorId).setCurrentPosition(0);
    man.motor(rightMotorId).setCurrentPosition(0);

    int ticks_M1 = 0;
    distance = distance / params.mmToTicks;

    while (ticks_M1 < distance) {
        while (stopRequested.load()) { 
            delay(10);
        }
        man.motor(leftMotorId).speed(speed);
        man.motor(rightMotorId).speed(-speed);

        man.motor(leftMotorId).requestInfo([&ticks_M1](rb::Motor& info) {
            ticks_M1 = info.position();
        });

        delay(10);
    }

    if (stopAtEnd) {
        // Stop motors at the end
        man.motor(leftMotorId).speed(0);
        man.motor(rightMotorId).speed(0);
    }
}

void Motor::turnLeft(int speed, int angle) {
    resetStopRequest();
    int targetPosition = angle * params.wheelDistance / (2 * M_PI * params.wheelRadius);
    targetPosition = targetPosition * params.mmToTicks;

    man.motor(leftMotorId).setCurrentPosition(0);
    man.motor(rightMotorId).setCurrentPosition(0);

    int currentPosition = 0;

    while (abs(currentPosition) < abs(targetPosition)) {
        while (stopRequested.load()) { 
            delay(10);
        }
        man.motor(leftMotorId).speed(-speed);
        man.motor(rightMotorId).speed(speed);

        man.motor(leftMotorId).requestInfo([&currentPosition](rb::Motor& info) {
            currentPosition = -info.position(); 
        });

        delay(10);
    }

    // Stop motors at the end
    man.motor(leftMotorId).speed(0);
    man.motor(rightMotorId).speed(0);
}

void Motor::turnRight(int speed, int angle) {
    resetStopRequest();
    int targetPosition = angle * params.wheelDistance / (2 * M_PI * params.wheelRadius);
    targetPosition = targetPosition * params.mmToTicks;

    man.motor(leftMotorId).setCurrentPosition(0);
    man.motor(rightMotorId).setCurrentPosition(0);

    man.motor(leftMotorId).speed(speed); 
    man.motor(rightMotorId).speed(-speed); 

    int currentPosition = 0;

    while (abs(currentPosition) < abs(targetPosition)) {
        while (stopRequested.load()) { 
            delay(10);
        }
        man.motor(leftMotorId).speed(speed);
        man.motor(rightMotorId).speed(-speed);

        man.motor(rightMotorId).requestInfo([&currentPosition](rb::Motor& info) {
            currentPosition = info.position(); 
        });

        delay(10);
    }

    // Stop motors at the end
    man.motor(leftMotorId).speed(0);
    man.motor(rightMotorId).speed(0);
}

void Motor::arcLeft(int speed, int angle, int radius) {
    resetStopRequest();
    int targetPosition = angle * params.wheelDistance / (2 * M_PI * params.wheelRadius);
    targetPosition = targetPosition * params.mmToTicks;

    man.motor(leftMotorId).setCurrentPosition(0);
    man.motor(rightMotorId).setCurrentPosition(0);

    int innerSpeed = speed * (radius - params.wheelDistance / 2) / (radius + params.wheelDistance / 2);
    int outerSpeed = speed;

    int currentPosition = 0;

    while (abs(currentPosition) < abs(targetPosition)) {
        while (stopRequested.load()) { 
            delay(10);
        }
        man.motor(leftMotorId).speed(innerSpeed);
        man.motor(rightMotorId).speed(outerSpeed);
        man.motor(leftMotorId).requestInfo([&currentPosition](rb::Motor& info) {
            currentPosition = -info.position(); 
        });

        delay(10);
    }

    // Stop motors at the end
    man.motor(leftMotorId).speed(0);
    man.motor(rightMotorId).speed(0);
}

void Motor::arcRight(int speed, int angle, int radius) {
    resetStopRequest();
    int targetPosition = angle * params.wheelDistance / (2 * M_PI * params.wheelRadius);
    targetPosition = targetPosition * params.mmToTicks;

    man.motor(leftMotorId).setCurrentPosition(0);
    man.motor(rightMotorId).setCurrentPosition(0);

    int innerSpeed = speed;
    int outerSpeed = speed * (radius - params.wheelDistance / 2) / (radius + params.wheelDistance / 2);

    int currentPosition = 0;

    while (abs(currentPosition) < abs(targetPosition)) {
        while (stopRequested.load()) { 
            delay(10);
        }
        man.motor(leftMotorId).speed(innerSpeed);
        man.motor(rightMotorId).speed(outerSpeed);

        man.motor(rightMotorId).requestInfo([&currentPosition](rb::Motor& info) {
            currentPosition = info.position(); 
        });

        delay(10);
    }

    // Stop motors at the end
    man.motor(leftMotorId).speed(0);
    man.motor(rightMotorId).speed(0);
}

void Motor::stop() {
    stopRequested.store(true);
    delay(10); // Wait for the stop request to be processed
    man.motor(leftMotorId).speed(0);
    man.motor(rightMotorId).speed(0);
}

void Motor::setMotorSpeed(int motorId, int speed, int durationMs) {
    auto startTime = std::chrono::steady_clock::now();
    bool keepRunning = true;

    while (keepRunning) {
        switch (motorId) {
        case 1:
            man.motor(rb::MotorId::M1).speed(speed);
            break;
        case 2:
            man.motor(rb::MotorId::M2).speed(speed);
            break;
        case 3:
            man.motor(rb::MotorId::M3).speed(speed);
            break;
        case 4:
            man.motor(rb::MotorId::M4).speed(speed);
            break;
        default:
            throw std::invalid_argument("Invalid motor ID");
        }

        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
        keepRunning = elapsed < durationMs;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    switch (motorId) {
    case 1:
        man.motor(rb::MotorId::M1).speed(0);
        break;
    case 2:
        man.motor(rb::MotorId::M2).speed(0);
        break;
    case 3:
        man.motor(rb::MotorId::M3).speed(0);
        break;
    case 4:
        man.motor(rb::MotorId::M4).speed(0);
        break;
    }
}

#endif // MOTOR_HPP
