#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "RBCX.h"
#include <Arduino.h>
#include <cmath>
#include <chrono>
#include <thread>

auto& man = rb::Manager::get();

// The Motor class represents a motor with basic movement capabilities.
/**
 * @brief The Motor class represents a motor in a robot.
 */
class Motor {
    std::atomic<bool> stopRequested{false}; 
public:
    // Structure to hold parameters
    /**
     * @brief The Parameters struct contains the parameters for the motor.
     */
    struct Parameters {
        int wheelDistance; /**< The distance between the wheels of the motor, in millimeters. */
        int wheelRadius; /**< The radius of the wheels, in millimeters. */
        int encoderTicksPerRevolution = 2000;

        float mmToTicks = encoderTicksPerRevolution / (2 * M_PI * wheelRadius); /**< Conversion factor from millimeters to encoder ticks. */
        float ticksToMm = 2.f * M_PI * wheelRadius / encoderTicksPerRevolution; /**< Conversion factor from encoder ticks to millimeters. */
    };

    void motorSetup() {
        man.install();
    }

    // Setter methods for motor IDs
    /**
     * @brief Sets the ID of the left motor.
     * @param id The ID of the left motor.
     */
    void setLeftMotorId(int id);
    
    /**
     * @brief Sets the ID of the right motor.
     * @param id The ID of the right motor.
     */
    void setRightMotorId(int id);

    // Basic movements
    /**
     * @brief Initiates forward movement of the motor at a specified speed and for a given distance.
     * @param speed The speed of the motor.
     * @param distance The distance to move.
     * @param stopAtEnd Flag indicating whether to stop the motor automatically at the end of the movement.
     */
    void moveForward(int speed, int distance, bool stopAtEnd = true);
    
    /**
     * @brief Moves the motor backward at a given speed.
     * @param speed The speed of the motor.
     * @param distance The distance to move.
     * @param stopAtEnd Flag indicating whether to stop the motor automatically at the end of the movement.
     */
    void moveBackward(int speed, int distance, bool stopAtEnd);
    
    /**
     * @brief Turns the motor left at a given speed.
     * @param speed The speed of the motor.
     * @param angle The angle to turn.
     */
    void turnLeft(int speed, int angle);
    
    /**
     * @brief Turns the motor right at a given speed.
     * @param speed The speed of the motor.
     * @param angle The angle to turn.
     */
    void turnRight(int speed, int angle);
    
    /**
     * @brief Makes the motor arc left at given inner and outer speeds.
     * @param speed The speed of the motor.
     * @param angle The angle to arc.
     * @param radius The radius of the arc.
     */
    void arcLeft(int speed, int angle, int radius);
    
    /**
     * @brief Makes the motor arc right at given inner and outer speeds.
     * @param speed The speed of the motor.
     * @param angle The angle to arc.
     * @param radius The radius of the arc.
     */
    void arcRight(int speed, int angle, int radius);
    
    /**
     * @brief Requests the motor to stop.
     */
    void requestStop() {
        stopRequested.store(true); // Set the stop flag
    }

    /**
     * @brief Resets the stop request flag at the start of a movement.
     */
    void resetStopRequest() {
        stopRequested.store(false); // Reset the stop flag at the start of a movement
    }

private:
    Parameters params;
    // Motor IDs for left and right motor.
    rb::MotorId leftMotorId;
    rb::MotorId rightMotorId;

    /**
     * @brief Sets the speed of the motor with a given ID.
     * @param motorId The ID of the motor.
     * @param speed The speed to set.
     * @param durationMs The duration for which to set the speed.
     */
    void setMotorSpeed(int motorId, int speed, int durationMs);
};

// Implementation

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
        // if(stopRequested.load()) { break; } 
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
