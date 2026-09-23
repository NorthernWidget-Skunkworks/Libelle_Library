// LibelleSWDemo.ino
#include <Libelle.h>

Libelle pyroUp(UP);

void setup() {
	Serial.begin(38400);
	if (!pyroUp.begin()) {
		Serial.print("Libelle not found: ");
		Serial.println(pyroUp.beginFailure());  // NoACK, NotSchema1, WrongName, OldFirmware, NoAccel
		while (1);
	}
	Serial.println(pyroUp.getHeader());
}

void loop() {
	Serial.println(pyroUp.getString());
	delay(1000);
}
