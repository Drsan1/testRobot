// #include "RoboBasement.hpp"
#include "Motor.hpp"

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
void Motor::moveStraight(int speed, int distance, bool stopAtEnd) {
    man.motor(leftMotorId).setCurrentPosition(0);
    man.motor(rightMotorId).setCurrentPosition(0);
    Motor::Parameters params;

    int ticks_M1 = 0;
    // int last_ticks_M1 = 0;
    // int ticks_M4 = 0;
    distance = distance / params.mmToTicks;

    while (ticks_M1 < distance) {
        man.motor(leftMotorId).speed(-speed);
        man.motor(rightMotorId).speed(speed);

        // man.motor(rb::MotorId::M4).requestInfo([&ticks_M4](rb::Motor &info) {
        //     ticks_M4 = info.position();
        // });

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

void Motor::moveBackward(int speed) {
};

void Motor::turnLeft(int speed) {
};

void Motor::turnRight(int speed) {
}

void Motor::arcLeft(int innerSpeed, int outerSpeed) {
}

void Motor::arcRight(int innerSpeed, int outerSpeed) {
}

void Motor::stop() {
}

// Sets the speed of the motor with a given ID.

void Motor::setMotorSpeed(int motorId, int speed) {
}