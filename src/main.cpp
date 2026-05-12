/* 

DHT22     -> GPIO 16
POT       -> GPIO 34
BUTTON 1  -> GPIO 18 -> EMERGENCY OVERRIDE (force system to reset/go in safe mode)
BUTTON 2  -> GPIO 19 -> MODE SWITCH (turn the system on or off)
RELAY     -> GPIO 26
BUZZER    -> GPIO 27
LED 1     -> GPIO 25
LED 2     -> GPIO 33

*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DHT.h"
#include "freertos/semphr.h"

// Defining our pin ports
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

DHT dht(DHTPIN, DHTTYPE);

typedef struct {
    float temperature;
    float humidity;
    int adcRaw;
    float setpoint;
    bool relayOn;
    bool alarmActive;
    int failureCount;
    float lastTemperature;
} SystemState_t;

SystemState_t systemState;
SemaphoreHandle_t stateMutex;

volatile bool button1Pressed = false;
volatile bool button2Pressed = false;
volatile unsigned long lastButton1Time = 0;
volatile unsigned long lastButton2Time = 0;

// Detects for any multiple attempts and only runs once
void IRAM_ATTR onButton1Press() {
    unsigned long now = millis();
   
    if (now - lastButton1Time > DEBOUNCE_MS) {
        button1Pressed = true;
        lastButton1Time = now;
    }
}

void IRAM_ATTR onButton2Press() {
        unsigned long now = millis();
   
    if (now - lastButton2Time > DEBOUNCE_MS) {
        button2Pressed = true;
        lastButton2Time = now;
    }
}


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

void buttonTask(void *pvParameters){
    while(1){
        if(button1Pressed){
            button1Pressed = false;  // reset flag
            Serial.println("Button 1: Emergency Override!");
            // reset the alarm
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            systemState.alarmActive = false;
            xSemaphoreGive(stateMutex);
            noTone(BUZZER_PIN);
            digitalWrite(LED_ALARM_PIN, LOW);
        }

        if (button2Pressed) {
            button2Pressed = false;  // reset flag
            Serial.println("Button 2: Mode Switch!");

            xSemaphoreTake(stateMutex, portMAX_DELAY);
            systemState.failureCount = 0;
            xSemaphoreGive(stateMutex);

            // toggles relay //
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            systemState.relayOn = !systemState.relayOn;
            digitalWrite(RELAY_PIN, systemState.relayOn);
            xSemaphoreGive(stateMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void safetyTask(void *pvParameters){
    while(1) {

        xSemaphoreTake(stateMutex, portMAX_DELAY);
        float temp = systemState.temperature;
        xSemaphoreGive(stateMutex);

        if(temp > OVERHEAT_THRESHOLD){
            //tone(BUZZER_PIN, 2000); 
            digitalWrite(RELAY_PIN, LOW); // Turn off our RELAY
            digitalWrite(LED_ALARM_PIN, HIGH); // Turn on our alarmss
            ledcWriteTone(0, 2000); // THIS MAKES A WARNING NOISE

            xSemaphoreTake(stateMutex, portMAX_DELAY);
            systemState.alarmActive = true;
            xSemaphoreGive(stateMutex);

            Serial.println("WARNING: Overheating detected");
        }
        else{
            // noTone(BUZZER_PIN); // //Turns off alarm
            ledcWriteTone(0, 0);
            digitalWrite(RELAY_PIN, HIGH); // Turn off our RELAY
            digitalWrite(LED_ALARM_PIN, LOW); // Turn on our alarmss

            xSemaphoreTake(stateMutex, portMAX_DELAY);
            systemState.alarmActive = false;
            xSemaphoreGive(stateMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void sensorTask(void *pvParameters) {
    while(1) {
        Serial.println("Reading sensor...");
        
        float temperature, humidity;
        xSemaphoreTake(stateMutex, portMAX_DELAY); //locking so one thread can access our data
        systemState.lastTemperature = systemState.temperature;

        systemState.humidity = dht.readHumidity();
        systemState.temperature = dht.readTemperature();


        // check for failed reading
        if (isnan(systemState.temperature) || isnan(systemState.humidity)) {
            systemState.failureCount++;
            xSemaphoreGive(stateMutex);
            Serial.println("ERROR: Sensor read failed!");

            // check if too many failures
            if (systemState.failureCount >= MAX_FAILURE) {
                Serial.println("CRITICAL: Too many failures - shutting down!");
                digitalWrite(RELAY_PIN, LOW);
                ledcWriteTone(0, 2000);
                digitalWrite(LED_ALARM_PIN, HIGH);
            }

            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Anomoly detection  
        if (abs(systemState.lastTemperature - systemState.temperature) > 10 )
        {
            Serial.println("WARNING: Anomoly detected - Rejecting sensor reading");
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            systemState.temperature = systemState.lastTemperature;
            xSemaphoreGive(stateMutex);
        }
        else{
            systemState.failureCount = 0;
        }
        
        xSemaphoreGive(stateMutex); // unlocking

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
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        systemState.adcRaw = readADC();
        systemState.setpoint = calibrateADC(systemState.adcRaw);
        xSemaphoreGive(stateMutex);

        Serial.print("ADC Raw: ");
        Serial.println(systemState.adcRaw);
        Serial.print("Setpoint: ");
        Serial.print(systemState.setpoint);
        Serial.println(" C");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() {
    stateMutex = xSemaphoreCreateMutex();
    Serial.begin(115200);
    dht.begin();

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_ALARM_PIN, OUTPUT);
    pinMode(LED_STATUS_PIN, OUTPUT);
    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);

    ledcAttachPin(BUZZER_PIN, 0);    // attach pin to channel 0
    ledcSetup(0, 2000, 8);



    attachInterrupt(digitalPinToInterrupt(18), onButton1Press, FALLING);
    attachInterrupt(digitalPinToInterrupt(19), onButton2Press, FALLING);

    xTaskCreate(buttonTask, "Buttons", 2048, NULL, 2, NULL);
    xTaskCreate(safetyTask, "Safety", 2048, NULL, 3, NULL);
    xTaskCreate(sensorTask, "Sensor", 2048, NULL, 2, NULL); // SENSOR TASKS
    xTaskCreate(adcTask, "ADC", 2048, NULL, 2, NULL); // TASKS
}

void loop() {
    // leave empty - FreeRTOS handles everything
}