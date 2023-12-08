#include "ProximitySleep.h"

#define APDS9960_INT PIN_INT_APDS  
#define LED_PIN 13                 // LED for showing interrupt


SparkFun_APDS9960 ProximitySleep::apds = SparkFun_APDS9960();
int ProximitySleep::isr_flag = 0;

void ProximitySleep::sleep(const uint8_t lowThreshold, const uint8_t highThreshold)
{
    for (int i = 2; i < 14; i++) pinMode(i, INPUT_PULLUP);
    // Set LED as output
    pinMode(LED_PIN, OUTPUT);
    pinMode(APDS9960_INT, INPUT);

    digitalWrite(LED_PIN, HIGH);



    /*Serial.println();
    Serial.println(F("---------------------------------------"));
    Serial.println(F("SparkFun APDS-9960 - ProximityInterrupt"));
    Serial.println(F("---------------------------------------"));
    */
    // Initialize interrupt service routine
    attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);

    // Initialize APDS-9960 (configure I2C and initial values)
    if (apds.init()) {
        Serial.println(F("APDS-9960 initialization complete"));
    } else {
        Serial.println(F("Something went wrong during APDS-9960 init!"));
    }

    // Adjust the Proximity sensor gain
    if (!apds.setProximityGain(PGAIN_4X)) {
        Serial.println(F("Something went wrong trying to set PGAIN"));
    }

    // Set proximity interrupt thresholds
    if (!apds.setProximityIntLowThreshold(lowThreshold)) {
        Serial.println(F("Error writing low threshold"));
    }
    if (!apds.setProximityIntHighThreshold(highThreshold)) {
        Serial.println(F("Error writing high threshold"));
    }

    // Start running the APDS-9960 proximity sensor (interrupts)
    if (apds.enableProximitySensor(true)) {
        Serial.println(F("Proximity sensor is now running"));
    } else {
        Serial.println(F("Something went wrong during sensor init!"));
    }

    apds.disableLightSensor();
    apds.disableGestureSensor();


    if (!apds.clearProximityInt()) {
        Serial.println("Error clearing interrupt");
    }

    //sd_app_evt_wait();
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PWR, LOW);
    pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
    pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);
    digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);
    digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);

    NRF_POWER->SYSTEMOFF = 1;
}

void ProximitySleep::interruptRoutine() {
    isr_flag = 1;
}