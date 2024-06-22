#ifndef ULTRASONIC_SENSOR_HPP
#define ULTRASONIC_SENSOR_HPP

#include <Arduino.h>

#define MAX_SENSOR_DISTANCE 200 // Maximum sensor distance can be as high as 500cm. Default=500
#define READ_INDIVIDUAL_DISTANCE true 

class UltrasonicSensor {
public:     
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin, unsigned int maxDistance = MAX_SENSOR_DISTANCE);

    double measureDistance();
    void calibrateSensor();
    bool isSafeDistance(double distance, double safeRange);
    void measureMultipleDistances(int numMeasurements, bool readIndividualDistance = READ_INDIVIDUAL_DISTANCE);
    double getAverageDistance() const;
    double getLowDistance() const;
    double getHighDistance() const;
    void handleSensorError();
    void sensorSelfTest();
    void measurementSleep(unsigned long sleepDuration);
    void shutdownSensor();

private:
    uint8_t trigPin_; // Pin for triggering the sensor
    uint8_t echoPin_; // Pin for receiving the echo signal
    bool isMeasurementEnabled_ = true; // Flag to control measurements
    int maxDistance_;

    double totalDistance;
    double avgDistance;
    double lowDistance;
    double highDistance;
    int numMeasurements;
};

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin, unsigned int maxDistance) {
    trigPin_ = trigPin;
    echoPin_ = echoPin;
    maxDistance_ = maxDistance; // Store the maximum allowed distance

    // Initialize GPIO pins and any sensor-specific settings here
    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
}

double UltrasonicSensor::measureDistance() {
    // Check if measurement is enabled
    if (!isMeasurementEnabled_) {
        return -1.0; // Return a negative value or error code to indicate disabled measurement
    }

    // Implement distance measurement logic using the trigPin_ and echoPin_
    // Return the distance in centimeters
    digitalWrite(trigPin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);
    
    long duration = pulseIn(echoPin_, HIGH);
    double distance = duration * 0.343 / 2;

    // Check if the measured distance exceeds the maximum allowed distance
    if (distance > maxDistance_) {
        return maxDistance_; // Return the maximum allowed distance
    }

    return distance;
}

void UltrasonicSensor::calibrateSensor() {
    // Calibration (Temperature)
}

bool UltrasonicSensor::isSafeDistance(double distance, double safeRange) {
    // Compare the measured distance to the safe range
    // Return true if within the safe range, false otherwise 
    return (distance < 0 || distance > safeRange);
}

void UltrasonicSensor::measureMultipleDistances(int numMeasurements, bool readIndividualDistance) {
    double totalDistance = 0;
    lowDistance = -1;
    highDistance = -1;
    avgDistance = 0;

    if (numMeasurements <= 0) {
        avgDistance = -1;  // Indicate no valid measurements
        return;
    }

    double minValidDistance = -1;  // Initialize the minimum valid distance
    double maxValidDistance = -1;  // Initialize the maximum valid distance

    for (int measurementIndex = 0; measurementIndex < numMeasurements; measurementIndex++) {
        double distance = measureDistance();

        if (distance >= 0) {
            if (readIndividualDistance) {
                // Serial.print("Measurement ");
                // Serial.print(measurementIndex + 1);
                // Serial.print(" - Distance: ");
                // Serial.print(distance);
                // Serial.println(" cm");
            }

            totalDistance += distance;

            if (minValidDistance < 0 || distance < minValidDistance) {
                minValidDistance = distance;
            }

            if (maxValidDistance < 0 || distance > maxValidDistance) {
                maxValidDistance = distance;
            }
        }
    }

    lowDistance = minValidDistance;
    highDistance = maxValidDistance;

    if (numMeasurements >= 5) {
        avgDistance = (totalDistance - minValidDistance - maxValidDistance) / (numMeasurements - 2);
    } else {
        avgDistance = totalDistance / numMeasurements;
    }
}

double UltrasonicSensor::getAverageDistance() const {
    return avgDistance;
}

double UltrasonicSensor::getLowDistance() const {
    return lowDistance;
}

double UltrasonicSensor::getHighDistance() const {
    return highDistance;
}

void UltrasonicSensor::handleSensorError() { 
    // Implement error handling logic here
    Serial.print("Ultrasonic sensor error: ");
    Serial.println("An error occurred while reading the sensor.");
}

void UltrasonicSensor::measurementSleep(unsigned long sleepDuration) {
    // Disable measurement by setting the measurement state flag to false
    isMeasurementEnabled_ = false;

    // Delay for the specified sleep duration
    delay(sleepDuration);

    // Re-enable measurement after the sleep duration
    isMeasurementEnabled_ = true;
}

void UltrasonicSensor::sensorSelfTest() {
    int maxDuration = 23200; // Microseconds (maximum range of 4 meters), default=23200
    int minDuration = 0;    // Default=0
    
    // Trigger the sensor
    digitalWrite(trigPin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);

    // Measure the echo time
    long duration = pulseIn(echoPin_, HIGH);

    // Check for reasonable values (adjust these values as needed)
    if (duration > minDuration && duration < maxDuration) {
        Serial.println("Sensor self-test passed. Working correctly.");
    } else {
        Serial.println("Sensor self-test failed. Not working as expected.");
    }
}

void UltrasonicSensor::shutdownSensor() {
    // Implement shutdown or cleanup logic here
    isMeasurementEnabled_ = false;
}

#endif // ULTRASONIC_SENSOR_HPP
