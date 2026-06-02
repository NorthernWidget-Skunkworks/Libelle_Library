#include "Margay.h"
#include <Libelle.h>

Libelle pyroUp(UP);
Libelle pyroDown(DOWN);

String Header = "";
uint8_t I2CVals[2] = {0x40, 0x41};
unsigned long UpdateRate = 60;

Margay Logger(Model_1v0);

void setup() {
	Header = Header + pyroUp.getHeader() + pyroDown.getHeader();
	Logger.begin(I2CVals, sizeof(I2CVals), Header);
	Init();
}

void loop() {
	Logger.Run(Update, UpdateRate);
}

String Update()
{
	Init();
	return pyroUp.getString() + pyroDown.getString();
}

void Init()
{
	pyroUp.begin();
	pyroDown.begin();
}
