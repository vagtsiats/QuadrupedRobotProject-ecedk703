#include <SPI.h>
#include <WiFiNINA.h>
#include "definitions.h"
using namespace BLA;

// #define TEST

const char* ssid = "wifiSSID";
const char* password = "wifiPass";

WiFiServer server(23);

unsigned long timer0;
float SPEED = 1.0;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("No WiFi module found!");
        while (true);
    }

    int status = WiFi.begin(ssid, password);
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        delay(10000);
        status = WiFi.begin(ssid, password);
    }

    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server.begin();

    digitalWrite(LED_BUILTIN, 1);
    Robot.initHardware();
    Robot.init_trot(2);

    delay(2000);

    timer0 = micros();
}

void loop() {
#ifdef TEST
    // Robot.drive_legs(conf, conf, conf, conf);
    // Robot.drive_legs_IK(pos, pos, pos, pos);
#else
    if (server.available()) { // If there is a client, read data
        while (client.connected()) {
            if (client.available()) {
                String input = client.readStringUntil('\n');
                Serial.println("Received: " + input);
                SPEED = input.toFloat();
                Serial.print("Updated SPEED: ");
                Serial.println(SPEED);
                Robot.init_trot(SPEED);
            }
        }
        client.stop();
    }
    
    if (micros() - timer0 >= LOOP_PERIODus) {
        timer0 = micros();

        Robot.walk(micros() / 1.e6, LOOP_PERIODsec);
    }
#endif
}
