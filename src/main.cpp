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
DHT dht(DHTPIN, DHTTYPE);



void sensorTask(void *pvParameters) {
    while(1) {
        Serial.println("Reading sensor...");

        float temperature, humidity;
        humidity = dht.readHumidity();
        temperature = dht.readTemperature();

        Serial.print("Humidity: ");
        Serial.println(humidity);
        Serial.print("Temperature: ");
        Serial.println(temperature);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void setup() {
    Serial.begin(115200);
    dht.begin();
    xTaskCreate(sensorTask, "Sensor", 2048, NULL, 2, NULL);
}

void loop() {
    // leave empty - FreeRTOS handles everything
}