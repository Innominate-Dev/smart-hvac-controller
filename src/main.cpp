#include "config.h"

DHT dht(DHTPIN, DHTTYPE);
SystemState_t systemState;
SemaphoreHandle_t stateMutex;

volatile bool button1Pressed = false;
volatile bool button2Pressed = false;
volatile unsigned long lastButton1Time = 0;
volatile unsigned long lastButton2Time = 0;

bool firstReading = true;

static bool ledState = false; 

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
int readRawADC() {
    int raw = analogRead(POT_PIN);  // reads 0-4095
    return raw;
}

int readADC(){
    int total = 0;
    int average = 0;

    for(int i =0; i < FILTER_SAMPLES; i++){
        total += analogRead(POT_PIN);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    average = total / FILTER_SAMPLES;
    return average;
}

float calibrateADC(int raw) {
    // converts raw ADC to temperature setpoint 16-30 degrees
    float calibrated = (raw / 4095.0) * (30.0 - 16.0) + 16.0;
    return calibrated;
}

void buttonTask(void *pvParameters){
    while(1){
        if(button1Pressed){
            button1Pressed = false;
            Serial.println("Button 1: Emergency Shutdown!");

            xSemaphoreTake(stateMutex, portMAX_DELAY);
            firstReading = true;
            systemState.failureCount = 0;
            systemState.alarmActive = true;
            systemState.systemShutdown = true;
            systemState.relayOn = false;
            xSemaphoreGive(stateMutex);

            digitalWrite(RELAY_PIN, LOW);
            ledcWriteTone(0, 2000);
            digitalWrite(LED_ALARM_PIN, HIGH);
        }

        if(button2Pressed){
            button2Pressed = false;
            Serial.println("Button 2: System Reset!");

            xSemaphoreTake(stateMutex, portMAX_DELAY);
            firstReading = true;
            systemState.failureCount = 0;
            systemState.alarmActive = false;
            systemState.systemShutdown = false;
            systemState.relayOn = true;
            xSemaphoreGive(stateMutex);

            digitalWrite(RELAY_PIN, HIGH);
            ledcWriteTone(0, 0);
            digitalWrite(LED_ALARM_PIN, LOW);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void safetyTask(void *pvParameters){
    esp_task_wdt_add(NULL);
    while(1) {
        if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE){

            if(systemState.systemShutdown){
                xSemaphoreGive(stateMutex);
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            float temp = systemState.temperature;
            xSemaphoreGive(stateMutex);

            if(temp > OVERHEAT_THRESHOLD){

                digitalWrite(RELAY_PIN, LOW); // Turn off our RELAY
                digitalWrite(LED_ALARM_PIN, HIGH); // Turn on our alarmss
                ledcWriteTone(0, 2000); // THIS MAKES A WARNING NOISE

                xSemaphoreTake(stateMutex, portMAX_DELAY);
                systemState.relayOn = false;
                systemState.alarmActive = true;
                xSemaphoreGive(stateMutex);

                Serial.println("WARNING: Overheating detected");
            }
            else{
                ledcWriteTone(0, 0); // Turns off Our ALARM
                digitalWrite(RELAY_PIN, HIGH); // Turn on our RELAY
                digitalWrite(LED_ALARM_PIN, LOW); // Turn off our LED alarmss

                xSemaphoreTake(stateMutex, portMAX_DELAY);
                if(!systemState.systemShutdown){systemState.relayOn = true;}
                systemState.alarmActive = false;
                xSemaphoreGive(stateMutex);
            }
            
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            if(systemState.alarmActive || systemState.systemShutdown){
                // alarm or shutdown - LED OFF
                digitalWrite(LED_STATUS_PIN, LOW);

            } else if(systemState.relayOn){
                // relay on, normal operation SOLID COLOUR
                digitalWrite(LED_STATUS_PIN, HIGH);

            } else {
                // relay manually switched off - START BLINKING
                ledState = !ledState;
                digitalWrite(LED_STATUS_PIN, ledState);
            }
            xSemaphoreGive(stateMutex);

        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void sensorTask(void *pvParameters) {
    
    while(1) {

        xSemaphoreTake(stateMutex, portMAX_DELAY);
        bool isShutdown = systemState.systemShutdown;
        xSemaphoreGive(stateMutex);

        if(isShutdown){
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        Serial.println("Reading sensor...");
        
        float temperature, humidity;
        xSemaphoreTake(stateMutex, portMAX_DELAY); //locking so one thread can access our data

        systemState.lastTemperature = systemState.temperature;

        systemState.humidity = dht.readHumidity();
        systemState.temperature = dht.readTemperature();

        if(firstReading){
            systemState.lastTemperature = systemState.temperature;
            firstReading = false;
        }
        

        // check for failed reading
        if (isnan(systemState.temperature) || isnan(systemState.humidity)) {
            systemState.failureCount++;
            Serial.println("ERROR: Sensor read failed!");

            // check if too many failures
            if (systemState.failureCount >= MAX_FAILURE) {
                Serial.println("CRITICAL: Too many failures - shutting down!");
                digitalWrite(RELAY_PIN, LOW);
                systemState.relayOn = false;
                systemState.systemShutdown = true;
                systemState.alarmActive = true;
                xSemaphoreGive(stateMutex);
                ledcWriteTone(0, 2000);
                digitalWrite(LED_ALARM_PIN, HIGH);
                continue;
            }

            xSemaphoreGive(stateMutex);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Anomoly detection  
        if (abs(systemState.lastTemperature - systemState.temperature) > 10 )
        {
            Serial.println("WARNING: Anomoly detected - Rejecting sensor reading");
            systemState.temperature = systemState.lastTemperature;
            systemState.failureCount++;

            if(systemState.failureCount >= MAX_FAILURE){
                Serial.println("CRITICAL: Too many anomalies - shutting down!");
                digitalWrite(RELAY_PIN, LOW);
                systemState.relayOn = false;
                systemState.systemShutdown = true;
                systemState.alarmActive = true;
                ledcWriteTone(0, 2000);
                digitalWrite(LED_ALARM_PIN, HIGH);
                xSemaphoreGive(stateMutex);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
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
        bool isShutdown = systemState.systemShutdown;
        xSemaphoreGive(stateMutex);

        if(isShutdown){
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        xSemaphoreTake(stateMutex, portMAX_DELAY);
        systemState.adcRaw = readRawADC();
        systemState.adcFiltered = readADC();
        systemState.setpoint = calibrateADC(systemState.adcFiltered);
        xSemaphoreGive(stateMutex);

        Serial.print("ADC Raw: ");
        Serial.println(systemState.adcRaw);
        Serial.print("ADC Filtered: ");
        Serial.println(systemState.adcFiltered);


        Serial.print("Setpoint: ");
        Serial.print(systemState.setpoint);
        Serial.println(" C");

        vTaskDelay(pdMS_TO_TICKS(1500));
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

    ledcSetup(0, 2000, 8);
    ledcAttachPin(BUZZER_PIN, 0);    // attach pin to channel 0

    systemState.systemShutdown = false;
    systemState.relayOn = true;
    systemState.alarmActive = false;
    systemState.failureCount = 0;

    esp_task_wdt_init(10, true);

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