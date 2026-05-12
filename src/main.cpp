/* 

DHT22     -> GPIO 16
POT       -> GPIO 34
BUTTON 1  -> GPIO 18
BUTTON 2  -> GPIO 19
RELAY     -> GPIO 26
BUZZER    -> GPIO 27
LED 1     -> GPIO 25
LED 2     -> GPIO 33

*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DHT.h"

#define DHTTYPE DHT22
#define DHTPIN 16
#define POT_PIN 34

DHT dht(DHTPIN, DHTTYPE);

typedef struct {
    float temperature;
    float humidity;
    int adcRaw;
    float setpoint;
    bool relayOn;
    bool alarmActive;
} SystemState_t;

SystemState_t systemState;

// reading whats set by the user via adc
int readADC() {
    int raw = analogRead(POT_PIN);  // reads 0-4095
    return raw;
}



float calibrateADC(int raw) {
    // converts raw ADC to temperature setpoint 16-30 degrees
    float calibrated = (raw / 4095.0) * (30.0 - 16.0) + 16.0;
    return calibrated;
}

void sensorTask(void *pvParameters) {
    while(1) {
        Serial.println("Reading sensor...");

        float temperature, humidity;

        systemState.humidity = dht.readHumidity();
        systemState.temperature = dht.readTemperature();

        if (isnan(systemState.temperature) || isnan(systemState.humidity)) 
        {
            Serial.println("ERROR: Sensor read failed!");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        Serial.print("Humidity: ");
        Serial.println(systemState.humidity);

        Serial.print("Temperature: ");
        Serial.print(systemState.temperature);
        Serial.println(" C");
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void adcTask(void *pvParameters) {
    while(1) {
        systemState.adcRaw = readADC();
        systemState.setpoint = calibrateADC(systemState.adcRaw);

        Serial.print("ADC Raw: ");
        Serial.println(systemState.adcRaw);
        Serial.print("Setpoint: ");
        Serial.print(systemState.setpoint);
        Serial.println(" C");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() {
    Serial.begin(115200);
    dht.begin();
    xTaskCreate(sensorTask, "Sensor", 2048, NULL, 2, NULL); // SENSOR TASKS
    xTaskCreate(adcTask, "ADC", 2048, NULL, 2, NULL); // TASKS
}

void loop() {
    // leave empty - FreeRTOS handles everything
}