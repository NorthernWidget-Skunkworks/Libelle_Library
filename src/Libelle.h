/******************************************************************************
Libelle.h
Library for interfacing with the Libelle shortwave pyranometer
Bobby Schulz @ Northern Widget LLC
7/5/2018
https://github.com/NorthernWidget-Skunkworks/Libelle_Library

Measures solar radiation across six spectral bands from UV-B through
short-wave infrared (~280-1700 nm), with tilt correction via onboard
accelerometer.

"The laws of nature are constructed in such a way as to make the universe
as interesting as possible."
-Freeman Dyson

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef Libelle_h
#define Libelle_h

#include <Arduino.h>
#include "Wire.h"
#include "math.h"

enum Orientation { UP = 0, DOWN = 1 };

/// Sentinel returned by getRoll() and getPitch() when the accelerometer is
/// unresponsive (all three axes return identical values).
constexpr float LIBELLE_ERROR = -9999.0f;

class Libelle
{
	public:
		Libelle(Orientation orientation = UP);
		bool begin();
		float getRoll();
		float getPitch();
		unsigned long getUVA();
		unsigned long getUVB();
		unsigned int getALS();
		unsigned int getWhite();
		float getLux();
		float getIR_Short();
		float getIR_Mid();
		float getTemp();
		String getHeader();
		String getString();

	private:
		const float LuxRes = 0.0036;

		const float A = 0.003354016;
		const float B = 0.0003074038;
		const float C = 1.019153E-05;
		const float D = 9.093712E-07;

		uint8_t ADR = 0x40;
		uint8_t Accel_ADR = 0x1D;
		Orientation _orientation = UP;
		bool initAccel();
		float getG(uint8_t Axis);
		float TempConvert(float V, float Vcc, float R, float A, float B, float C, float D, float R25);
		void PrintAllRegs();
		bool WriteByte(uint8_t Adr, uint8_t Pos, uint8_t Val);
		uint8_t ReadByte(uint8_t Adr, uint8_t Pos);
		unsigned int ReadWord(uint8_t Adr, uint8_t Pos);
};

#endif
