#include "RoboBasement.hpp"
#include "SmartServoBus.hpp"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <iostream>
#include <thread>

const char* ssid = "ESP32";
const char* password = "123456789";

WiFiServer server(23);

using namespace lx16a;

static SmartServoBus servoBus;
Motor motor;

const int numSensors = 2;
uint32_t maxDistance = 200;

UltrasonicSensor ultrasonicSensors[numSensors] = {
    UltrasonicSensor(3, 2, maxDistance),
    UltrasonicSensor(4, 19, maxDistance),
};

bool stopRequested = false;
void Collision() {
    int numMeasurements = 5;

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

        delay(100);
    }
}

void setup() {
    Serial.begin(115200);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    server.begin();

    Serial.println("Connected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    motor.motorSetup();

    Motor::Parameters params;
    params.wheelRadius = 5;
    params.wheelDistance = 10;

    servoBus.begin(1, UART_NUM_2, GPIO_NUM_14);

    //////////////MOVEMENT//////////////
    std::thread collisionThread(Collision);
    collisionThread.detach();

    motor.setLeftMotorId(1);
    motor.setRightMotorId(2);

    motor.moveForward(100, 9999);

    servoBus.set(0, 180_deg);
}

void loop() {
    WiFiClient client = server.available();

    if (client) {
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                Serial.write(c);
            }

            client.print(stopRequested);
        }
        // close the connection:
        client.stop();
        Serial.println("Client Disconnected.");
    }
}