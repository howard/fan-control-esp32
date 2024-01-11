#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#include "config.h"

#define PIN_FAN0_TACH 32
#define PIN_FAN0_PWM 27
#define SILENT_TEMPERATURE_THRESHOLD 22
#define PWM_FREQUENCY 25000
#define PWM_RESOLUTION 8
#define PWM_CHAN_FAN0 0

WiFiClient espClient;
PubSubClient pubSubClient(espClient);
Adafruit_BMP280 bmp;

int rotationCounter;
int fan0speed = 50;
bool fan0auto = true;

void reconnectNetwork() {
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }

    Serial.print("Connecting to network");
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
        Serial.print(".");
    }
    Serial.println("!");
    Serial.println(WiFi.localIP());
}

void reconnectPubSub() {
    if (pubSubClient.connected()) {
        return;
    }

    Serial.print("Connecting to MQTT");
    while (!pubSubClient.connected()) {
        if (pubSubClient.connect("ESP32FanController")) {
            Serial.println("!");
            delay(500);
            pubSubClient.subscribe("rack-fan-ctrl/fan0/speed");
            pubSubClient.subscribe("rack-fan-ctrl/fan0/auto");
            Serial.println("Subscribed to rack-fan-ctrl/fan0/speed.");
        } else {
            Serial.print(".");
        }
        delay(1000);
    }
}

void IRAM_ATTR incrementRotationCount() {
    rotationCounter++;
}


int readFanRpm(int tachoPin) {
    rotationCounter = 1;
    attachInterrupt(tachoPin, incrementRotationCount, RISING);
    delay(250);
    detachInterrupt(tachoPin);

    return (rotationCounter / 2.0) * 240.0;
}

void setFanSpeed(int fanPwmChannel, int speedPercent) {
    int newPwmDuty = (int)(255 * (speedPercent / 100.0));
    ledcWrite(fanPwmChannel, newPwmDuty);
}

void handleMqttMessage(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, "rack-fan-ctrl/fan0/speed") == 0) {
        int newSpeedPercent = atoi(std::string((char*)payload, length).c_str());
        setFanSpeed(PWM_CHAN_FAN0, newSpeedPercent);
        fan0auto = false;
        pubSubClient.publish("rack-fan-ctrl/fan0/auto", "false");
        Serial.printf("Setting fan speed of fan0 to %d.\n", newSpeedPercent);
        return;
    }

    if (strcmp(topic, "rack-fan-ctrl/fan0/auto") == 0) {
        bool newAutoMode = strcmp(std::string((char*)payload, length).c_str(), "true") == 0;
        fan0auto = newAutoMode;
        Serial.printf("Toggling fan0 auto mode to %d.\n", fan0auto);
        return;
    }
}

void adjustFanSpeedToTemperature(int fanPwmChannel, float temperature) {
    if (temperature < SILENT_TEMPERATURE_THRESHOLD) {
        setFanSpeed(fanPwmChannel, 0);
        return;
    }

    // Go to 100% speed at 15C above silence threshold.
    // Make sure not to go beyond 100%.
    int newSpeed = min(100, (int)(((temperature - SILENT_TEMPERATURE_THRESHOLD) / 15.0) * 100));
    Serial.printf("New auto speed is %d.\n", newSpeed);
    setFanSpeed(fanPwmChannel, newSpeed);
}

void setup()
{
    Wire.begin();
    
    Serial.begin(115200);

    if (!bmp.begin(0x76)) {
        Serial.println("Failed to connect to BMP280.");
    }

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    delay(1000);

    reconnectNetwork();

    pubSubClient.setServer(MQTT_SERVER, 1883);
    pubSubClient.setCallback(handleMqttMessage);

    reconnectPubSub();

    pinMode(PIN_FAN0_TACH, INPUT_PULLUP);
    ledcSetup(PWM_CHAN_FAN0, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_FAN0_PWM, PWM_CHAN_FAN0);
    ledcWrite(PWM_CHAN_FAN0, 255);

    pubSubClient.publish("rack-fan-ctrl/fan0/auto", "true");

    Serial.println("Init done.");
}


void loop()
{
    reconnectNetwork();
    reconnectPubSub();

    pubSubClient.loop();

    static char strTemperatureCelsius[7];
    float temperature = bmp.readTemperature();
    dtostrf(temperature, 6, 2, strTemperatureCelsius);
    pubSubClient.publish("rack-fan-ctrl/temperature", strTemperatureCelsius);

    static char strPressurePa[7];
    dtostrf(bmp.readPressure(), 6, 2, strPressurePa);
    pubSubClient.publish("rack-fan-ctrl/pressure", strPressurePa);

    static char strFan0Rpm[7];
    dtostrf(readFanRpm(PIN_FAN0_TACH), 6, 2, strFan0Rpm);
    pubSubClient.publish("rack-fan-ctrl/fan0/rpm", strFan0Rpm);

    if (fan0auto) {
        adjustFanSpeedToTemperature(PWM_CHAN_FAN0, temperature);
    }

    Serial.printf("%s Temp(C) %s Pressure(Pa) %s Fan0(rpm)\n", strTemperatureCelsius, strPressurePa, strFan0Rpm);

    delay(250);
}