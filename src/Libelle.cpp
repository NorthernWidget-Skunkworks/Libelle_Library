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

// Schema 1 Page 2 register addresses (NW-Device-Specification, Libelle appendix)
static constexpr uint8_t ALS_ADR      = 0x48;  // Block 1, VEML6030: ALS, white, lux multiplier (uint16 each)
static constexpr uint8_t WHITE_ADR    = 0x4A;
static constexpr uint8_t LUXMUL_ADR   = 0x4C;
static constexpr uint8_t UVA_ADR      = 0x50;  // Block 2, VEML6075: UVA, UVB (int32 each)
static constexpr uint8_t UVB_ADR      = 0x54;
static constexpr uint8_t IR_SHORT_ADR = 0x58;  // Block 3, ADS1115: IR short, IR mid, thermistor (uint16 each)
static constexpr uint8_t IR_MID_ADR   = 0x5A;
static constexpr uint8_t THERM_ADR    = 0x5C;

// ADXL343 accelerometer data register addresses
static constexpr uint8_t XAXIS = 0x32;
static constexpr uint8_t YAXIS = 0x34;
static constexpr uint8_t ZAXIS = 0x36;

Libelle::Libelle(Orientation orientation)
{
  _orientation = orientation;
  if(_orientation == DOWN) {
    Accel_ADR = 0x53;
  }
  else {
    Accel_ADR = 0x1D;
  }
}

bool Libelle::begin(uint8_t ADR_)
{
  if(ADR_ == 0) ADR_ = (_orientation == DOWN) ? DEFAULT_ADDRESS_DOWN : DEFAULT_ADDRESS_UP;
  // Page 0 gates: Schema 1, the name "Libelle", firmware patch >= LIBELLE_FW_MIN_PATCH.
  _bridgeOk = _dev.begin(ADR_, "Libelle", LIBELLE_FW_MIN_PATCH);
  _accelOk = initAccel();
  delay(2); // ADXL343 standby→measurement startup time: 1.4 ms max
  return _bridgeOk && _accelOk;
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

float Libelle::getRoll(bool update)
{
  if(update) updateMeasurements(ADXL343);
  return _roll;
}

float Libelle::getPitch(bool update)
{
  if(update) updateMeasurements(ADXL343);
  return _pitch;
}

long Libelle::getUVA(bool update)
{
  if(update) updateMeasurements(VEML6075);
  return _uva;
}

long Libelle::getUVB(bool update)
{
  if(update) updateMeasurements(VEML6075);
  return _uvb;
}

long Libelle::getALS(bool update)
{
  if(update) updateMeasurements(VEML6030);
  return _als;
}

long Libelle::getWhite(bool update)
{
  if(update) updateMeasurements(VEML6030);
  return _white;
}

float Libelle::getLux(bool update)
{
  if(update) updateMeasurements(VEML6030);
  return _lux;
}

float Libelle::getIR_Short(bool update)
{
  if(update) updateMeasurements(ADS1115);
  return _irShort;
}

float Libelle::getIR_Mid(bool update)
{
  if(update) updateMeasurements(ADS1115);
  return _irMid;
}

float Libelle::getTemp(bool update)
{
  if(update) updateMeasurements(ADS1115);
  return _temp;
}

float Libelle::TempConvert(float V, float Vcc, float R, float A, float B, float C, float D, float R25)
{
  float Rt = (Vcc/V)*R - R;
  float LogRt = log(Rt/R25);
  float T = 1.0/(A + B*LogRt + C*pow(LogRt, 2.0) + D*pow(LogRt, 3.0));
  return T;
}

bool Libelle::updateMeasurements(uint8_t component)
{
  resetReadings(component);
  if((component & BRIDGE) == BRIDGE && _uvCfg.n <= 1 && _lightCfg.n <= 1 && _irCfg.n <= 1) {
    // One reading of everything on the bridge: one trigger, one 22-byte read.
    _dev.resetBatch();
    if(_dev.takeReading(BRIDGE)) readData();
  }
  else {
    // Per chip group: N readings each, appended to the arrays; a chip that
    // reports absent (not answering / self-test failed) stops its batch.
    if(component & VEML6075) _dev.takeReadings(VEML6075, _uvCfg.n, [this] { return updateUV(); });
    if(component & VEML6030) _dev.takeReadings(VEML6030, _lightCfg.n, [this] { return updateLight(); });
    if(component & ADS1115) _dev.takeReadings(ADS1115, _irCfg.n, [this] { return updateIR(); });
  }
  if(component & ADXL343) {
    // Hardware v1: the accelerometer answers the controller directly, so no
    // batch word; an unresponsive accelerometer costs one reading, not N.
    for(uint16_t i = 0; i < _tiltCfg.n; i++) if(!updateTilt()) break;
  }
  summarise(component);
  bool ok = true;
  if(component & VEML6075) ok = ok && _uvaReadings.count() > 0;
  if(component & VEML6030) ok = ok && _alsReadings.count() > 0;
  if(component & ADS1115) ok = ok && _irShortReadings.count() > 0;
  if(component & ADXL343) ok = ok && _rollReadings.count() > 0;
  return ok;
}

bool Libelle::updateUV()
{
  uint8_t d[8];
  if(!_dev.takeReading(VEML6075) || !_dev.readData(UVA_ADR, d, 8)) return false;
  return readUV(d);
}

bool Libelle::updateLight()
{
  uint8_t d[6];
  if(!_dev.takeReading(VEML6030) || !_dev.readData(ALS_ADR, d, 6)) return false;
  return readLight(d);
}

bool Libelle::updateIR()
{
  uint8_t d[6];
  if(!_dev.takeReading(ADS1115) || !_dev.readData(IR_SHORT_ADR, d, 6)) return false;
  return readIR(d);
}

bool Libelle::updateTilt()
{
  float ValX = getG(0);
  float ValY = getG(1);
  float ValZ = getG(2);
  _accelFault = (ValX == ValY && ValX == ValZ); // unresponsive: every axis reads the same
  if(_accelFault) return false;
  _rollReadings.append(atan(ValX / sqrt(pow(ValY, 2) + pow(ValZ, 2))) * (180.0 / M_PI));
  _pitchReadings.append(atan(ValY / sqrt(pow(ValX, 2) + pow(ValZ, 2))) * (180.0 / M_PI));
  return true;
}

static uint16_t word16(const uint8_t* d) { return d[0] | (d[1] << 8); }
static int32_t word32(const uint8_t* d) { return (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24)); }

bool Libelle::readUV(uint8_t* d)
{
  if(_dev.faulted(0)) return false; // VEML6075: UVA, UVB int32 compensated counts
  _uvaReadings.append(word32(d));
  _uvbReadings.append(word32(d + 4));
  return true;
}

bool Libelle::readLight(uint8_t* d)
{
  if(_dev.faulted(1)) return false; // VEML6030: ALS, white, auto-range multiplier, uint16 each
  uint16_t als = word16(d), white = word16(d + 2), luxMul = word16(d + 4);
  _alsReadings.append(als);
  _whiteReadings.append(white);
  _luxReadings.append(float(als)*float(luxMul)*LuxRes);
  return true;
}

bool Libelle::readIR(uint8_t* d)
{
  if(_dev.faulted(2)) return false; // ADS1115: IR short, IR mid, thermistor, uint16 raw counts
  _irShortReadings.append(word16(d));
  _irMidReadings.append(word16(d + 2));
  float Val = float(word16(d + 4))*(1.25e-4);
  _tempReadings.append(TempConvert(Val, 3.3, 10000.0, A, B, C, D, 10000.0) - 273.15);
  return true;
}

bool Libelle::readData()
{
  // Blocks 1-3 are consecutive (0x48-0x5D): one read.
  uint8_t d[22];
  if(!_dev.readData(NW_REG_DATA, d, 22)) return false;
  readLight(d);
  readUV(d + 8);
  readIR(d + 16);
  return true;
}

void Libelle::resetReadings(uint8_t component)
{
  if(component & VEML6075) { _uvaReadings.reset(); _uvbReadings.reset(); }
  if(component & VEML6030) { _alsReadings.reset(); _whiteReadings.reset(); _luxReadings.reset(); }
  if(component & ADS1115) { _irShortReadings.reset(); _irMidReadings.reset(); _tempReadings.reset(); }
  if(component & ADXL343) { _rollReadings.reset(); _pitchReadings.reset(); }
}

void Libelle::summarise(uint8_t component)
{
  // Means over the readings taken; LIBELLE_ERROR when none (its rounded mean
  // is itself). Counts are rounded; IR is scaled from counts to volts.
  if(component & VEML6075) {
    _uva = lround(_uvaReadings.mean());
    _uvb = lround(_uvbReadings.mean());
  }
  if(component & VEML6030) {
    _als   = lround(_alsReadings.mean());
    _white = lround(_whiteReadings.mean());
    _lux   = _luxReadings.mean();
  }
  if(component & ADS1115) {
    _irShort = nwScaled(_irShortReadings.mean(), 8000.0); // 1.25e-4 V per count
    _irMid   = nwScaled(_irMidReadings.mean(), 8000.0);
    _temp    = _tempReadings.mean();
  }
  if(component & ADXL343) {
    _roll  = _rollReadings.mean();
    _pitch = _pitchReadings.mean();
  }
}

uint16_t Libelle::setUVReadings(uint16_t n)    { return _uvCfg.set(n, LIBELLE_UV_CAPACITY); }
uint16_t Libelle::setLightReadings(uint16_t n) { return _lightCfg.set(n, LIBELLE_LIGHT_CAPACITY); }
uint16_t Libelle::setIRReadings(uint16_t n)    { return _irCfg.set(n, LIBELLE_IR_CAPACITY); }
uint16_t Libelle::setTiltReadings(uint16_t n)  { return _tiltCfg.set(n, LIBELLE_TILT_CAPACITY); }
void     Libelle::setUVStats(bool enable)      { _uvCfg.stats = enable; }
void     Libelle::setLightStats(bool enable)   { _lightCfg.stats = enable; }
void     Libelle::setIRStats(bool enable)      { _irCfg.stats = enable; }
void     Libelle::setTiltStats(bool enable)    { _tiltCfg.stats = enable; }
uint16_t Libelle::getUVCount()    { return _uvaReadings.count(); }
uint16_t Libelle::getLightCount() { return _alsReadings.count(); }
uint16_t Libelle::getIRCount()    { return _irShortReadings.count(); }
uint16_t Libelle::getTiltCount()  { return _rollReadings.count(); }

// Statistics are computed from the arrays each call (NW_Readings), in the
// units stored: counts for UVA, UVB, ALS and white; lux, degrees C and degrees
// for the rest; IR counts are scaled to volts. LIBELLE_ERROR when empty.
static float volts(float v) { return nwScaled(v, 8000.0); }
float Libelle::getUVAMean()        { return _uvaReadings.mean(); }
float Libelle::getUVAStd()         { return _uvaReadings.std(); }
float Libelle::getUVASterr()       { return _uvaReadings.sterr(); }
float Libelle::getUVAMedian()      { return _uvaReadings.median(); }
float Libelle::getUVBMean()        { return _uvbReadings.mean(); }
float Libelle::getUVBStd()         { return _uvbReadings.std(); }
float Libelle::getUVBSterr()       { return _uvbReadings.sterr(); }
float Libelle::getUVBMedian()      { return _uvbReadings.median(); }
float Libelle::getALSMean()        { return _alsReadings.mean(); }
float Libelle::getALSStd()         { return _alsReadings.std(); }
float Libelle::getALSSterr()       { return _alsReadings.sterr(); }
float Libelle::getALSMedian()      { return _alsReadings.median(); }
float Libelle::getWhiteMean()      { return _whiteReadings.mean(); }
float Libelle::getWhiteStd()       { return _whiteReadings.std(); }
float Libelle::getWhiteSterr()     { return _whiteReadings.sterr(); }
float Libelle::getWhiteMedian()    { return _whiteReadings.median(); }
float Libelle::getLuxMean()        { return _luxReadings.mean(); }
float Libelle::getLuxStd()         { return _luxReadings.std(); }
float Libelle::getLuxSterr()       { return _luxReadings.sterr(); }
float Libelle::getLuxMedian()      { return _luxReadings.median(); }
float Libelle::getIR_ShortMean()   { return volts(_irShortReadings.mean()); }
float Libelle::getIR_ShortStd()    { return volts(_irShortReadings.std()); }
float Libelle::getIR_ShortSterr()  { return volts(_irShortReadings.sterr()); }
float Libelle::getIR_ShortMedian() { return volts(_irShortReadings.median()); }
float Libelle::getIR_MidMean()     { return volts(_irMidReadings.mean()); }
float Libelle::getIR_MidStd()      { return volts(_irMidReadings.std()); }
float Libelle::getIR_MidSterr()    { return volts(_irMidReadings.sterr()); }
float Libelle::getIR_MidMedian()   { return volts(_irMidReadings.median()); }
float Libelle::getTempMean()       { return _tempReadings.mean(); }
float Libelle::getTempStd()        { return _tempReadings.std(); }
float Libelle::getTempSterr()      { return _tempReadings.sterr(); }
float Libelle::getTempMedian()     { return _tempReadings.median(); }
float Libelle::getRollMean()       { return _rollReadings.mean(); }
float Libelle::getRollStd()        { return _rollReadings.std(); }
float Libelle::getRollSterr()      { return _rollReadings.sterr(); }
float Libelle::getRollMedian()     { return _rollReadings.median(); }
float Libelle::getPitchMean()      { return _pitchReadings.mean(); }
float Libelle::getPitchStd()       { return _pitchReadings.std(); }
float Libelle::getPitchSterr()     { return _pitchReadings.sterr(); }
float Libelle::getPitchMedian()    { return _pitchReadings.median(); }

String Libelle::column(const char* name, const char* unit, bool stats)
{
  // "R_u [deg]," for the UP unit, "R_d [deg]," for DOWN; with stats the std
  // and sterr columns follow: "R_u std [deg],R_u sterr [deg],".
  String Name = String(name) + (_orientation == DOWN ? "_d" : "_u");
  String Header = Name + unit + ",";
  if(stats) Header += Name + " std" + unit + "," + Name + " sterr" + unit + ",";
  return Header;
}

String Libelle::getHeader()
{
  bool st = _tiltCfg.columns(), su = _uvCfg.columns(), sl = _lightCfg.columns(), si = _irCfg.columns();
  return column("R", " [deg]", st) + column("P", " [deg]", st) + column("UVA", "", su) + column("UVB", "", su)
       + column("White", "", sl) + column("Vis", " [lx]", sl)
       + column("IR_S", "", si) + column("IR_M", "", si) + column("PyroT", " [C]", si);
}

String Libelle::getString()
{
  updateMeasurements(ALL);
  bool st = _tiltCfg.columns(), su = _uvCfg.columns(), sl = _lightCfg.columns(), si = _irCfg.columns();
  String s = String(getRoll()) + ",";
  if(st) s += String(getRollStd()) + "," + String(getRollSterr()) + ",";
  s += String(getPitch()) + ",";
  if(st) s += String(getPitchStd()) + "," + String(getPitchSterr()) + ",";
  s += String(getUVA()) + ",";
  if(su) s += String(getUVAStd()) + "," + String(getUVASterr()) + ",";
  s += String(getUVB()) + ",";
  if(su) s += String(getUVBStd()) + "," + String(getUVBSterr()) + ",";
  s += String(getWhite()) + ",";
  if(sl) s += String(getWhiteStd()) + "," + String(getWhiteSterr()) + ",";
  s += String(getLux()) + ",";
  if(sl) s += String(getLuxStd()) + "," + String(getLuxSterr()) + ",";
  s += String(getIR_Short()) + ",";
  if(si) s += String(getIR_ShortStd()) + "," + String(getIR_ShortSterr()) + ",";
  s += String(getIR_Mid()) + ",";
  if(si) s += String(getIR_MidStd()) + "," + String(getIR_MidSterr()) + ",";
  s += String(getTemp()) + ",";
  if(si) s += String(getTempStd()) + "," + String(getTempSterr()) + ",";
  return s;
}

// The reading interface: one reading per logReading(), printed as it is taken.
void Libelle::beginReadings(uint8_t component, uint16_t n)
{
  _component = component;
  resetReadings(component);
  if(component & BRIDGE) _dev.beginBatch(n);
  else _dev.resetBatch(); // the accelerometer alone: nothing to tell the bridge
}

void Libelle::endReadings()
{
  // No cleanup required currently
}

size_t Libelle::printHeader(Print& out)
{
  size_t n = 0;
  if(_component & ADXL343)  n += out.print(column("R", " [deg]", false) + column("P", " [deg]", false));
  if(_component & VEML6075) n += out.print(column("UVA", "", false) + column("UVB", "", false));
  if(_component & VEML6030) n += out.print(column("White", "", false) + column("Vis", " [lx]", false));
  if(_component & ADS1115)  n += out.print(column("IR_S", "", false) + column("IR_M", "", false) + column("PyroT", " [C]", false));
  return n;
}

size_t Libelle::printReading(Print& out)
{
  size_t n = 0;
  if(_component & ADXL343)  { n += out.print(_roll); n += out.print(','); n += out.print(_pitch); n += out.print(','); }
  if(_component & VEML6075) { n += out.print(_uva); n += out.print(','); n += out.print(_uvb); n += out.print(','); }
  if(_component & VEML6030) { n += out.print(_white); n += out.print(','); n += out.print(_lux); n += out.print(','); }
  if(_component & ADS1115)  { n += out.print(_irShort); n += out.print(','); n += out.print(_irMid); n += out.print(','); n += out.print(_temp); n += out.print(','); }
  return n;
}

size_t Libelle::logReading(Print& out)
{
  // One acquisition per chip group selected, then the values just taken.
  if(_component & ADXL343) {
    _roll = _pitch = LIBELLE_ERROR;
    if(updateTilt()) { _roll = _rollReadings.last(); _pitch = _pitchReadings.last(); }
  }
  if(_component & VEML6075) {
    _uva = _uvb = (long)LIBELLE_ERROR;
    if(updateUV()) { _uva = _uvaReadings.last(); _uvb = _uvbReadings.last(); }
  }
  if(_component & VEML6030) {
    _als = _white = (long)LIBELLE_ERROR; _lux = LIBELLE_ERROR;
    if(updateLight()) { _als = _alsReadings.last(); _white = _whiteReadings.last(); _lux = _luxReadings.last(); }
  }
  if(_component & ADS1115) {
    _irShort = _irMid = _temp = LIBELLE_ERROR;
    if(updateIR()) { _irShort = volts(_irShortReadings.last()); _irMid = volts(_irMidReadings.last()); _temp = _tempReadings.last(); }
  }
  return printReading(out);
}

bool    Libelle::faulted(uint8_t chip) { return (chip == 3) ? _accelFault : _dev.faulted(chip); } // chip 3, the ADXL343, is the library's own on hardware v1
bool    Libelle::anyFault()            { return _dev.anyFault() || _accelFault; }
uint8_t Libelle::reportChip()           { return report().chip(); }
uint8_t Libelle::reportKind()           { return report().kind(); }
uint8_t Libelle::getHardwareMajor()    { return _dev.hardwareMajor(); }
uint8_t Libelle::getHardwareMinor()    { return _dev.hardwareMinor(); }
uint8_t Libelle::getFirmwareVersion()  { return _dev.firmwareVersion(); }

NW_Report Libelle::report()
{
  // The bridge's report first. Hardware v1: the accelerometer is the
  // library's to judge; not answering at begin() is kind 1, three identical
  // axes afterwards kind 4 (out of range).
  if(_dev.reportKind() == 0 && _accelFault) {
    NW_Report f;
    f.code = (3 << 5) | (_accelOk ? 4 : 1);
    return f;
  }
  return _dev.report();
}

String Libelle::beginFailure()
{
  if(_bridgeOk && !_accelOk) return "NoAccel"; // the bridge passed; the accelerometer did not acknowledge
  return _dev.beginFailure();
}

// The chip names are Libelle's own (the spec's chip table); NW_Report prints the rest.
static const char* const chips[] = {"VEML6075", "VEML6030", "ADS1115", "ADXL343"};

size_t Libelle::printReport(Print& out)
{
  return report().print(out, chips, 4);
}

size_t Libelle::printStatus(Print& out, bool boot)
{
  return _dev.printSnapshot(out, chips, 4, boot);
}

bool    Libelle::reportIsFault()   { return report().isFault(); } // the accelerometer's own fault counts too
uint8_t Libelle::bootReportKind()  { return _dev.bootReport().kind(); }
void    Libelle::clearBootReport() { _dev.clearBootReport(); }

String Libelle::reportNote()
{
  // One word for a data-table note: the chip, then the kind ("VEML6075NotAnswering").
  return report().note(chips, 4);
}

void Libelle::PrintAllRegs()
{
  uint8_t Regs[96]; // Page 0 (identity), Page 1 (calibration) and Page 2 (status, control, data)
  _dev.readBytes(0x00, Regs, 96);
  for(int i = 0; i < 96; i++) {
    Serial.print("Reg"); Serial.print(i, HEX); Serial.print(":\t");
    Serial.println(Regs[i]);
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
