// LibelleSWDemo.ino
#include <Libelle.h>

Libelle pyroUp(UP);

void setup() {
	Serial.begin(38400);
	if (!pyroUp.begin()) {
		Serial.println("Libelle not found. Check wiring.");
		while (1);
	}
	Serial.println(pyroUp.getHeader());
}

void loop() {
	Serial.println(pyroUp.getString());
	delay(1000);
}
