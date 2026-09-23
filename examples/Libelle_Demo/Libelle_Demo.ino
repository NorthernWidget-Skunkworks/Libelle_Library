#include <Libelle.h>

Libelle pyro;   // facing UP: bridge at 0x4C, accelerometer at 0x1D

void setup() {
    Serial.begin(9600);
    Serial.println("Libelle shortwave pyranometer");
    Serial.println("Measures UV-B, UV-A, visible, near-IR, tilt, and temperature.");

    if (!pyro.begin()) {
        Serial.print("Libelle not found: ");
        Serial.println(pyro.beginFailure());  // NoACK, NotSchema1, WrongName, OldFirmware, NoAccel
        while (1);
    }

    Serial.println(pyro.getHeader());
}

void loop() {
    Serial.println(pyro.getString());  // -9999 where a reading failed
    if (pyro.anyFault()) {
        pyro.printReport(Serial);  // e.g. "VEML6075: no acknowledge"
        Serial.println();
    }
    delay(1000);
}
