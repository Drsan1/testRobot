#include "RoboBasement.hpp"
#include <iostream>
#include <thread>
#include <chrono> 

Motor motor;

void Collision() { 
    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // Add us sensor here
    motor.stop();
}

void setup() {
    Motor::Parameters params;
    params.wheelRadius = 5;
    params.wheelDistance = 10;

    std::thread collisionThread(Collision); 
    collisionThread.detach(); 

    motor.setLeftMotorId(1);
    motor.setRightMotorId(2);

    motor.moveForward(100, 9999);
}

void loop() {
    
}