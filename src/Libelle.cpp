/******************************************************************************
Libelle.cpp
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

#include "Wire.h"
#include "math.h"
#include "Libelle.h"

// ATtiny841 bridge register addresses
static constexpr uint8_t UVA_ADR      = 0x02;
static constexpr uint8_t UVB_ADR      = 0x06;
static constexpr uint8_t ALS_ADR      = 0x0B;
static constexpr uint8_t WHITE_ADR    = 0x0D;
static constexpr uint8_t LUXMUL_ADR   = 0x10;
static constexpr uint8_t IR_MID_ADR   = 0x13;
static constexpr uint8_t IR_SHORT_ADR = 0x15;
static constexpr uint8_t THERM_ADR    = 0x17;

// ADXL343 accelerometer data register addresses
static constexpr uint8_t XAXIS = 0x32;
static constexpr uint8_t YAXIS = 0x34;
static constexpr uint8_t ZAXIS = 0x36;

Libelle::Libelle(Orientation orientation)
{
  _orientation = orientation;
  if(_orientation == DOWN) {
    Accel_ADR = 0x53;
    ADR = 0x41;
  }
  else {
    Accel_ADR = 0x1D;
    ADR = 0x40;
  }
}

bool Libelle::begin()
{
	Wire.begin();
  Wire.beginTransmission(ADR);
  bool sensorOk = (Wire.endTransmission() == 0);
  bool accelOk = initAccel();
  delay(2); // ADXL343 standby→measurement startup time: 1.4 ms max
  return sensorOk && accelOk;
}

bool Libelle::initAccel()
{
  bool ok = true;
  ok &= WriteByte(Accel_ADR, 0x2D, 0x08);
  ok &= WriteByte(Accel_ADR, 0x31, 0x08);
  ok &= WriteByte(Accel_ADR, 0x38, 0x00);
  ok &= WriteByte(Accel_ADR, 0x2C, 0x0A);
  return ok;
}

float Libelle::getG(uint8_t Axis)
{
  WriteByte(Accel_ADR, 0x2D, 0x08);
  Wire.beginTransmission(Accel_ADR);
  Wire.write(XAXIS + 2*Axis);
  Wire.endTransmission();
  Wire.requestFrom(Accel_ADR, 2);
  int LSB = Wire.read();
  int MSB = Wire.read();
  float g = ((MSB << 8) | LSB)*(0.0039);
  return g;
}

float Libelle::getRoll()
{
  float ValX = getG(0);
  float ValY = getG(1);
  float ValZ = getG(2);
  if(ValX == ValY && ValX == ValZ) return LIBELLE_ERROR;
  return atan(ValX / sqrt(pow(ValY, 2) + pow(ValZ, 2))) * (180.0 / M_PI);
}

float Libelle::getPitch()
{
  float ValX = getG(0);
  float ValY = getG(1);
  float ValZ = getG(2);
  if(ValX == ValY && ValX == ValZ) return LIBELLE_ERROR;
  return atan(ValY / sqrt(pow(ValX, 2) + pow(ValZ, 2))) * (180.0 / M_PI);
}

long Libelle::getUVA()
{
  long LSW = ReadWord(ADR, UVA_ADR);
  long MSW = ReadWord(ADR, UVA_ADR + 2);
  return (MSW << 16) | LSW;
}

long Libelle::getUVB()
{
  long LSW = ReadWord(ADR, UVB_ADR);
  long MSW = ReadWord(ADR, UVB_ADR + 2);
  return (MSW << 16) | LSW;
}

unsigned int Libelle::getALS()
{
	return ReadWord(ADR, ALS_ADR);
}

unsigned int Libelle::getWhite()
{
  return ReadWord(ADR, WHITE_ADR);
}

float Libelle::getLux()
{
  return float(ReadWord(ADR, ALS_ADR))*float(ReadWord(ADR, LUXMUL_ADR))*LuxRes;
}

float Libelle::getIR_Short()
{
  return float(ReadWord(ADR, IR_SHORT_ADR))*(1.25e-4);
}

float Libelle::getIR_Mid()
{
  return float(ReadWord(ADR, IR_MID_ADR))*(1.25e-4);
}

float Libelle::getTemp()
{
  float Val = float(ReadWord(ADR, THERM_ADR))*(1.25e-4);
  return TempConvert(Val, 3.3, 10000.0, A, B, C, D, 10000.0) - 273.15;
}

float Libelle::TempConvert(float V, float Vcc, float R, float A, float B, float C, float D, float R25)
{
  float Rt = (Vcc/V)*R - R;
  float LogRt = log(Rt/R25);
  float T = 1.0/(A + B*LogRt + C*pow(LogRt, 2.0) + D*pow(LogRt, 3.0));
  return T;
}

String Libelle::getHeader()
{
	String Header = "";
	if(_orientation == UP)   Header = "R_u [deg],P_u [deg],UVA_u,UVB_u,White_u,Vis_u [lx],IR_S_u,IR_M_u,PyroT_u [C],";
	if(_orientation == DOWN) Header = "R_d [deg],P_d [deg],UVA_d,UVB_d,White_d,Vis_d [lx],IR_S_d,IR_M_d,PyroT_d [C],";
	return Header;
}

String Libelle::getString()
{
	return String(getRoll()) + "," + String(getPitch()) + "," + String(getUVA()) + "," + String(getUVB()) + "," + String(getWhite()) + "," + String(getLux()) + "," + String(getIR_Short()) + "," + String(getIR_Mid()) + "," + String(getTemp()) + ",";
}

void Libelle::PrintAllRegs()
{
  for(int i = 0; i < 0x1A; i++) {
    Serial.print("Reg"); Serial.print(i, HEX); Serial.print(":\t");
    Serial.println(ReadByte(ADR, i));
  }
  Serial.print("\n\n");
}

bool Libelle::WriteByte(uint8_t Adr, uint8_t Pos, uint8_t Val)
{
  Wire.beginTransmission(Adr);
  Wire.write(Pos);
  Wire.write(Val);
  return Wire.endTransmission() == 0;
}

uint8_t Libelle::ReadByte(uint8_t Adr, uint8_t Pos)
{
  Wire.beginTransmission(Adr);
  Wire.write(Pos);
  Wire.endTransmission(true);
  Wire.requestFrom(Adr, 1);
  return Wire.read();
}

unsigned int Libelle::ReadWord(uint8_t Adr, uint8_t Pos)
{
  unsigned int LSB = ReadByte(Adr, Pos);
  unsigned int MSB = ReadByte(Adr, Pos + 1);
  return (MSB << 8) | LSB;
}
