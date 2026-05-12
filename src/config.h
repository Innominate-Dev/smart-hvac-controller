#ifndef CONFIG_H
#define CONFIG_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "DHT.h"

#define DHTTYPE DHT22
#define DHTPIN 16
#define POT_PIN 34
#define RELAY_PIN 26
#define BUZZER_PIN 27
#define LED_ALARM_PIN 33
#define LED_STATUS_PIN 25
#define OVERHEAT_THRESHOLD 35.0
#define DEBOUNCE_MS 300
#define MAX_FAILURE 3
#define FILTER_SAMPLES 5

typedef struct {
    float temperature;
    float humidity;
    int adcFiltered;
    int adcRaw;
    float setpoint;
    bool relayOn;
    bool alarmActive;
    bool systemShutdown;
    int failureCount;
    float lastTemperature;
} SystemState_t;

// just declarations - defined in main.cpp
extern SystemState_t systemState;
extern SemaphoreHandle_t stateMutex;
extern bool firstReading;
extern DHT dht;

#endif