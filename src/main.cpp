#include "Motor.hpp" // Include the Motor class definition
#include <iostream>

Motor motor; // Create a Motor object

void setup() {
    Motor::Parameters params; // Create a Parameters object
    params.wheelRadius = 5; // Set the wheel radius
    params.wheelDistance = 10; // Set the distance between wheels

    motor.setParameters(params); // Apply parameters to the motor

    motor.setLeftMotorId(1); // Set the left motor ID
    motor.setRightMotorId(2); // Set the right motor ID

    delay(1000);
    // motor.moveStraight(100, 250);

}

void loop() {
    // Put your main code here, to run repeatedly:
    // For example, motor.driveForward();
}