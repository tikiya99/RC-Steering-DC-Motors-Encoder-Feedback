#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define PPM_PIN 15 // PPM input pin
#define PPM_SYNC_PULSE_WIDTH 3000 

// Motor control pins
const int encoderPinA1 = 33, encoderPinB1 = 32; // Inner wheel encoder pins
const int LPWM1 = 19, RPWM1 = 21;               // Inner wheel motor control pins
const int encoderPinA2 = 26, encoderPinB2 = 25; // Outer wheel encoder pins
const int LPWM2 = 22, RPWM2 = 23;               // Outer wheel motor control pins

const float wheelbase = 0.6, trackWidth = 0.6;  // Robot geometry constants
const int encoderResolution = 1800;             // Encoder resolution for angle calculation

volatile long encoderPulses1 = 0, encoderPulses2 = 0;
float currentAngle1 = 0, currentAngle2 = 0, desiredAngle1 = 0, desiredAngle2 = 0;
float initialAngle1 = 0, initialAngle2 = 0; // Initial positions

float targetAngle = 0;

volatile uint16_t ppmValues[8]; 
volatile uint8_t currentChannel = 0;
volatile uint32_t lastPPMTime = 0;
volatile bool ppmUpdated = false;

void IRAM_ATTR handlePPMInterrupt();
void IRAM_ATTR handleEncoder1();
void IRAM_ATTR handleEncoder2();
void stopMotor(int LPWM, int RPWM);
void adjustMotorSpeed(int LPWM, int RPWM, float angleDifference);
void processPPM();
void processEncoder(long encoderPulses, float &currentAngle, float desiredAngle, int LPWM, int RPWM);

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    pinMode(encoderPinA1, INPUT_PULLUP);
    pinMode(encoderPinB1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA1), handleEncoder1, CHANGE);

    pinMode(LPWM1, OUTPUT);
    pinMode(RPWM1, OUTPUT);

    pinMode(encoderPinA2, INPUT_PULLUP);
    pinMode(encoderPinB2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA2), handleEncoder2, CHANGE);

    pinMode(LPWM2, OUTPUT);
    pinMode(RPWM2, OUTPUT);

    pinMode(PPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), handlePPMInterrupt, FALLING);

    Serial.println("Setup complete. Waiting for commands.");
}

void loop() {
    if (ppmUpdated) {
        processPPM();
        ppmUpdated = false;
    }

    processEncoder(encoderPulses1, currentAngle1, desiredAngle1, LPWM1, RPWM1);
    processEncoder(encoderPulses2, currentAngle2, desiredAngle2, LPWM2, RPWM2);

    delay(10);
}

void IRAM_ATTR handlePPMInterrupt() {
    uint32_t currentTime = micros();
    uint32_t pulseWidth = currentTime - lastPPMTime;
    lastPPMTime = currentTime;

    if (pulseWidth > PPM_SYNC_PULSE_WIDTH) {
        currentChannel = 0;
    } else {
        if (currentChannel < 8) {
            ppmValues[currentChannel] = pulseWidth;
            currentChannel++;
            if (currentChannel == 8) {
                ppmUpdated = true;
            }
        }
    }
}

void IRAM_ATTR handleEncoder1() {
    encoderPulses1 += (digitalRead(encoderPinA1) == digitalRead(encoderPinB1)) ? 1 : -1;
}

void IRAM_ATTR handleEncoder2() {
    encoderPulses2 += (digitalRead(encoderPinA2) == digitalRead(encoderPinB2)) ? 1 : -1;
}

void stopMotor(int LPWM, int RPWM) {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);
    Serial.println("Motor stopped.");
}

void adjustMotorSpeed(int LPWM, int RPWM, float angleDifference) {
    int motorSpeed = 150; // Constant motor speed for motor turning
    if (angleDifference > 0) {
        analogWrite(LPWM, motorSpeed);
        analogWrite(RPWM, 0);
    } else {
        analogWrite(LPWM, 0);
        analogWrite(RPWM, motorSpeed);
    }
    Serial.print("Adjusting motor speed for angle: ");
    Serial.println(angleDifference, 2);
}


void processPPM() {
    int ppmValue = ppmValues[0]; // Read the PPM value for channel 1
   
    /*if (ppmValue < 1480) {
        targetAngle = -30;
    } else if (ppmValue <= 1520) {
        targetAngle = 0;
    } else {
        targetAngle = 30;
    }

    float innerRadius = (wheelbase / tan(targetAngle * PI / 180)) - (trackWidth / 2);
    float outerRadius = (wheelbase / tan(targetAngle * PI / 180)) + (trackWidth / 2);
    desiredAngle1 = atan(wheelbase / innerRadius) * (180 / PI);
    desiredAngle2 = atan(wheelbase / outerRadius) * (180 / PI);*/


    if (ppmValue >= 1480 && ppmValue <= 1520) {
        desiredAngle1 = initialAngle1;
        desiredAngle2 = initialAngle2;
        Serial.println("PPM Value within stop range: Returning to initial positions.");
    } else {
        float targetAngle;
        if (ppmValue < 1480) {
            targetAngle = map(ppmValue, 1000, 1480, 20, 0); 
        } else if (ppmValue > 1520) {
            targetAngle = map(ppmValue, 1520, 2000, 0, -20); 
        } else {
            targetAngle = 0;
        }

        // Truncate the target angle to an integer
        targetAngle = int(targetAngle);

        float innerRadius = (wheelbase / tan(targetAngle * PI / 180.0)) - (trackWidth / 2.0); 
        float outerRadius = (wheelbase / tan(targetAngle * PI / 180.0)) + (trackWidth / 2.0); 
        desiredAngle1 = atan(wheelbase / innerRadius) * (180.0 / PI);
        desiredAngle2 = atan(wheelbase / outerRadius) * (180.0 / PI);
        
        Serial.print("Target Angle: ");
        Serial.println(targetAngle);
        Serial.print("Desired Inner Angle: ");
        Serial.println(desiredAngle1);
        Serial.print("Desired Outer Angle: ");
        Serial.println(desiredAngle2);
    }
}

void processEncoder(long encoderPulses, float &currentAngle, float desiredAngle, int LPWM, int RPWM) {
    long desiredPulses = (long)((desiredAngle / 360.0) * encoderResolution);
    currentAngle = (encoderPulses / (float)encoderResolution) * 360;
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);

    if (abs(encoderPulses) >= abs(desiredPulses)) {
        stopMotor(LPWM, RPWM);
    } else {
        float angleDifference = desiredAngle - currentAngle;
        adjustMotorSpeed(LPWM, RPWM, angleDifference);
    }
}
