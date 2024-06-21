#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "RBCX.h"
#include <cmath>
#include <Arduino.h>

// The Motor class represents a motor with basic movement capabilities.
class Motor {
public:
    // Structure to hold parameters
    struct Parameters {
        int wheelDistance;
        int wheelRadius; // in mm
        int encoderTicksPerRevolution = 2000;

        // Removed direct initialization
        float mmToTicks;
        float ticksToMm;

        // Constructor to ensure proper initialization
        Parameters() : wheelDistance(0), wheelRadius(0), mmToTicks(0), ticksToMm(0) {}

        // Method to update dependent values
        void updateCalculations() {
            mmToTicks = encoderTicksPerRevolution / (2 * M_PI * wheelRadius);
            ticksToMm = 2.f * M_PI * wheelRadius / encoderTicksPerRevolution;
        }
    };

    // Setter methods for motor IDs
    void setLeftMotorId(int id);
    void setRightMotorId(int id);

    // Method to set parameters and update calculations
    void setParameters(const Parameters& params) {
        this->params = params;
        this->params.updateCalculations();
    }

    // Basic movements
    // Moves the motor straight at a given speed.
    void moveStraight(int speed, int distance, bool stopAtEnd = true);
    // Moves the motor backward at a given speed.
    void moveBackward(int speed);
    // Turns the motor left at a given speed.
    void turnLeft(int speed);
    // Turns the motor right at a given speed.
    void turnRight(int speed);
    // Makes the motor arc left at given inner and outer speeds.
    void arcLeft(int innerSpeed, int outerSpeed);
    // Makes the motor arc right at given inner and outer speeds.
    void arcRight(int innerSpeed, int outerSpeed);
    // Stops the motor.
    void stop();

private:
    Parameters params;
    // Motor IDs for left and right motor.
    rb::MotorId leftMotorId;
    rb::MotorId rightMotorId;

    // Sets the speed of the motor with a given ID.
    void setMotorSpeed(int motorId, int speed);
};

#endif // MOTOR_HPP