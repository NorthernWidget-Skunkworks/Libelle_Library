#include <Libelle.h>

Libelle pyro;

void setup() {
    Serial.begin(9600);
    Serial.println("Libelle shortwave pyranometer");
    Serial.println("Measures UV-B, UV-A, visible, near-IR, tilt, and temperature.");

    if (!pyro.begin()) {
        Serial.println("Libelle not found. Check wiring.");
        while (1);
    }

    Serial.println(pyro.getHeader());
}

void loop() {
    Serial.println(pyro.getString());
    delay(1000);
}
