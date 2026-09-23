// Output-regression test for Libelle_Library: compiles src/Libelle.cpp against
// the NW_Core stubs and prints getHeader()/getString()/getters for fixed
// register images. Two chips share the bus: the pyranometer bridge (Schema 1
// register map) and the ADXL343 accelerometer, which the library reads
// directly (hardware v1). run.sh diffs the result against baseline.txt.
#include "Arduino.h"
#include "Wire.h"
TwoWire Wire;
#include "../../src/Libelle.cpp"
#include "NW_TestSupport.h"

// The ADXL343: a register file of its own at 0x1D (UP) or 0x53 (DOWN), served by the hooks.
static uint8_t adxl[64];
static uint8_t adxlAddress = 0x1D;
static bool adxlPresent = true;
static void setAccel(int16_t x, int16_t y, int16_t z) {   // 0.0039 g per LSB: 256 = 1 g
  adxl[0x32] = x & 0xFF; adxl[0x33] = (x >> 8) & 0xFF;
  adxl[0x34] = y & 0xFF; adxl[0x35] = (y >> 8) & 0xFF;
  adxl[0x36] = z & 0xFF; adxl[0x37] = (z >> 8) & 0xFF;
}
static void installBus(uint8_t bridgeAddress) {
  Wire.deviceAddress = bridgeAddress;
  Wire.isPresent = [](uint8_t a) { return Wire.present && (a == Wire.deviceAddress || (adxlPresent && a == adxlAddress)); };
  Wire.hookOnly  = [](uint8_t a) { return a == adxlAddress; };
  Wire.onRequest = [](TwoWire& w, uint8_t n, std::deque<uint8_t>& out) {
    if (w.address() != adxlAddress) return false;                       // the bridge: the image
    for (uint8_t i = 0; i < n; i++) out.push_back(adxl[(w.pointer() + i) & 0x3F]);
    return true;
  };
  installFirmwareEmulation();
}

// Schema 1 register image: Page 0 as NW-Provision writes it (with the firmware's
// patch at 0x0A), Page 1 (calibration) zero, Page 2 with a complete reading (Libelle
// appendix: VEML6030 ALS, white and lux multiplier uint16 at 0x48; VEML6075 UVA and
// UVB int32 at 0x50; ADS1115 IR short, IR mid and thermistor uint16 at 0x58), little-endian.
static void put16(uint8_t* r, uint8_t at, uint16_t v) { r[at] = v & 0xFF; r[at + 1] = v >> 8; }
static void put32(uint8_t* r, uint8_t at, int32_t v) { for (int i = 0; i < 4; i++) r[at + i] = (v >> (8 * i)) & 0xFF; }
static void loadImage(int32_t uva, int32_t uvb, uint16_t als, uint16_t white, uint16_t luxMul,
                      uint16_t irMid, uint16_t irShort, uint16_t therm, uint8_t fwPatch = 1, uint8_t schema = 0x01) {
  uint8_t* r = Wire.image;
  nwLoadPage0(r, "Libelle", Wire.deviceAddress, 1, fwPatch, schema);   // Page 0 and Block 0, HW 0.1
  put16(r, 0x48, als); put16(r, 0x4A, white); put16(r, 0x4C, luxMul);
  put32(r, 0x50, uva); put32(r, 0x54, uvb);
  put16(r, 0x58, irShort); put16(r, 0x5A, irMid); put16(r, 0x5C, therm);
}
static void loadStandard() { loadImage(12345, 678, 4000, 3000, 8, 20000, 10000, 13200); }

static void report(const char* name, Libelle& s) {
  printf("[%s]\n", name);
  printf("header: %s\n", s.getHeader().c_str());
  printf("string: %s\n", s.getString().c_str());
  printf("getters: uva=%ld uvb=%ld als=%ld white=%ld lux=%.4f irS=%.4f irM=%.4f temp=%.4f roll=%.4f pitch=%.4f\n",
         s.getUVA(), s.getUVB(), s.getALS(), s.getWhite(), s.getLux(), s.getIR_Short(), s.getIR_Mid(), s.getTemp(), s.getRoll(), s.getPitch());
}

int main() {
  // 1. UP unit, level: ALS 4000 x mult 8 x 0.0036 = 115.2 lx; IR 1.25 V and 2.5 V; thermistor at 25.00 C
  //    (1.65 V: Rt = R25, so the Steinhart-Hart log term is zero); 1 g on Z.
  installBus(0x4C); adxlAddress = 0x1D; setAccel(0, 0, 256);
  loadStandard();
  { Libelle s(UP); printf("begin=%d\n", s.begin()); report("UP, level", s); }

  // 2. Tilted: X 100, Y 50, Z 230 LSB.
  setAccel(100, 50, 230);
  { Libelle s(UP); s.begin(); report("UP, tilted", s); }

  // 3. DOWN unit: bridge at 0x0C, accelerometer at 0x53; a different set of values.
  installBus(0x0C); adxlAddress = 0x53; setAccel(-30, 10, 250);
  loadImage(1000000, 2000000, 65535, 1, 1, 65535, 0, 2640);   // 0.33 V on the thermistor
  { Libelle s(DOWN); printf("begin=%d\n", s.begin()); report("DOWN", s); }

  // 4. Nothing on the bus.
  installBus(0x4C); adxlAddress = 0x1D; setAccel(0, 0, 256); loadStandard(); Wire.present = false;
  { Libelle s(UP); bool ok = s.begin(); printf("begin with no sensor: %d failure=%s\n", ok, s.beginFailure().c_str()); report("absent", s); }
  Wire.present = true;

  // 5. begin() gates: wrong name, wrong schema, firmware too old, the versions it reports,
  //    an explicit address, and a bridge without its accelerometer.
  loadStandard(); Wire.image[0x01] = 'X';
  { Libelle s; bool ok = s.begin(); printf("[wrong name] begin=%d failure=%s\n", ok, s.beginFailure().c_str()); }
  loadImage(12345, 678, 4000, 3000, 8, 20000, 10000, 13200, 1, 0x00);
  { Libelle s; bool ok = s.begin(); printf("[schema 0x00] begin=%d failure=%s\n", ok, s.beginFailure().c_str()); }
  loadImage(12345, 678, 4000, 3000, 8, 20000, 10000, 13200, 0);
  { Libelle s; bool ok = s.begin(); printf("[fw patch 0 < min %d] begin=%d fw=%u failure=%s\n", LIBELLE_FW_MIN_PATCH, ok, s.getFirmwareVersion(), s.beginFailure().c_str()); }
  loadStandard();
  { Libelle s; bool ok = s.begin(); printf("[versions] begin=%d hw=%u.%u fw=%u failure=%s\n", ok, s.getHardwareMajor(), s.getHardwareMinor(), s.getFirmwareVersion(), s.beginFailure().c_str()); }
  { Libelle s(DOWN); bool ok = s.begin(0x4C); printf("[DOWN unit at an explicit 0x4C] begin=%d failure=%s\n", ok, s.beginFailure().c_str()); }
  adxlPresent = false;
  { Libelle s; bool ok = s.begin(); printf("[no accelerometer] begin=%d failure=%s\n", ok, s.beginFailure().c_str());
    String row = s.getString();   // evaluated before the fault getters: printf argument order is unspecified
    printf("[no accelerometer] string: %s faulted(3)=%d any=%d note='%s'\n", row.c_str(), s.faulted(3), s.anyFault(), s.reportNote().c_str()); }
  adxlPresent = true;

  // 6. Faults: the VEML6075 does not acknowledge (status bit 1, pan-fault, latched 0x01); the
  //    other chips survive. Then an ADS1115 timeout (0x22), then a unit reset with a clean
  //    status, then an accelerometer whose three axes read the same (the library's own fault).
  loadStandard();
  { Libelle s; s.begin(); char pb[48];
    onReading = [](TwoWire& w) { w.image[0x40] = 0x83; w.image[0x47] = 0x01; };
    bool ok = s.updateMeasurements(); BufferPrint bp(pb, sizeof pb); s.printReport(bp);
    printf("[VEML6075 no ACK] update=%d faulted(0)=%d faulted(1)=%d faulted(2)=%d any=%d chip=%u kind=%u text='%s' note='%s'\n",
           ok, s.faulted(0), s.faulted(1), s.faulted(2), s.anyFault(), s.reportChip(), s.reportKind(), pb, s.reportNote().c_str());
    printf("[VEML6075 no ACK] string: %s\n", s.getString().c_str());
    onReading = [](TwoWire& w) { w.image[0x40] = 0x89; w.image[0x47] = 0x42; };   // chip 2, kind 2
    String row = s.getString();   // evaluated before the note: printf argument order is unspecified
    printf("[ADS1115 timeout] string: %s note='%s'\n", row.c_str(), s.reportNote().c_str());
    onReading = [](TwoWire& w) { w.image[0x40] = 0x01; w.image[0x47] = 0xE6; };
    ok = s.updateMeasurements(); BufferPrint bp2(pb, sizeof pb); s.printReport(bp2);
    printf("[unit reset] update=%d any=%d chip=%u kind=%u text='%s' note='%s'\n", ok, s.anyFault(), s.reportChip(), s.reportKind(), pb, s.reportNote().c_str());
    onReading = nullptr; setAccel(0, 0, 0);
    ok = s.updateMeasurements(); BufferPrint bp3(pb, sizeof pb); s.printReport(bp3);
    printf("[accelerometer flat] update=%d faulted(3)=%d any=%d chip=%u kind=%u text='%s' note='%s' roll=%.2f tiltCount=%u\n", ok, s.faulted(3), s.anyFault(), s.reportChip(), s.reportKind(), pb, s.reportNote().c_str(), s.getRoll(), s.getTiltCount());
    setAccel(0, 0, 256); }

  // 7. N readings with statistics: ALS steps through five values (lux follows), UVA through
  //    three, the thermistor through two; the batch word reaches the bridge; the
  //    accelerometer takes four; getString() grows its columns.
  loadStandard();
  { Libelle s; s.begin(); int k = 0;
    onReading = [&](TwoWire& w) { k++;
      put16(w.image, 0x48, 4000 + 100 * (k % 5));
      put32(w.image, 0x50, 12345 + 10 * (k % 3));
      put16(w.image, 0x5C, 13200 + 40 * (k % 2)); };
    printf("[N] setLightReadings(5)=%u setUVReadings(3)=%u setIRReadings(2)=%u setTiltReadings(4)=%u setLightReadings(99)=%u\n",
           s.setLightReadings(5), s.setUVReadings(3), s.setIRReadings(2), s.setTiltReadings(4), s.setLightReadings(99));
    s.setLightReadings(5); s.setLightStats(true); s.setUVStats(true); s.setIRStats(true); s.setTiltStats(true);
    lastRequest = 0; unsigned t0 = Wire.transactions; bool ok = s.updateMeasurements();
    printf("[N=5,3,2,4] update=%d lightCount=%u uvCount=%u irCount=%u tiltCount=%u lastRequest=%u requestFrom=%u\n",
           ok, s.getLightCount(), s.getUVCount(), s.getIRCount(), s.getTiltCount(), lastRequest, Wire.transactions - t0);
    printf("[N=5,3,2,4] als mean=%.4f std=%.4f sterr=%.4f median=%.4f | lux mean=%.4f std=%.4f | uva mean=%.4f std=%.4f median=%.4f | temp mean=%.4f std=%.4f | roll mean=%.4f std=%.4f\n",
           s.getALSMean(), s.getALSStd(), s.getALSSterr(), s.getALSMedian(), s.getLuxMean(), s.getLuxStd(),
           s.getUVAMean(), s.getUVAStd(), s.getUVAMedian(), s.getTempMean(), s.getTempStd(), s.getRollMean(), s.getRollStd());
    printf("[N=5,3,2,4] header: %s\n", s.getHeader().c_str());
    printf("[N=5,3,2,4] string: %s\n", s.getString().c_str());
    ok = s.updateMeasurements(Libelle::VEML6030);
    printf("[VEML6030 only] update=%d lightCount=%u uvCount=%u uva=%ld lux=%.4f\n", ok, s.getLightCount(), s.getUVCount(), s.getUVA(), s.getLux());
    onReading = nullptr; }

  // 8. Reading interface: header, three logged readings of ALL, then the accelerometer
  //    alone (nothing written to the bridge), then the VEML6030 alone.
  loadStandard();
  { Libelle s; s.begin(); int k = 0; char pb[160];
    onReading = [&](TwoWire& w) { k++; put32(w.image, 0x50, 12300 + 5 * k); };
    lastRequest = 0; s.beginReadings(Libelle::ALL, 3);
    BufferPrint bh(pb, sizeof pb); s.printHeader(bh); printf("[run ALL] header: %s lastRequest=%u\n", pb, lastRequest);
    for (int i = 0; i < 3; i++) { BufferPrint bp(pb, sizeof pb); size_t n = s.logReading(bp); printf("[run ALL] row %d (%zu bytes): %s\n", i, n, pb); }
    s.endReadings();
    printf("[run ALL] uva count=%u mean=%.4f median=%.4f\n", s.getUVCount(), s.getUVAMean(), s.getUVAMedian());
    lastRequest = 0; unsigned t0 = Wire.transactions; s.beginReadings(Libelle::ADXL343, 2);
    BufferPrint bh2(pb, sizeof pb); s.printHeader(bh2); printf("[run ADXL343] header: %s lastRequest=%u\n", pb, lastRequest);
    BufferPrint bp2(pb, sizeof pb); s.logReading(bp2); s.endReadings(); printf("[run ADXL343] row: %s requestFrom=%u\n", pb, Wire.transactions - t0);
    s.beginReadings(Libelle::VEML6030);
    BufferPrint bh3(pb, sizeof pb); s.printHeader(bh3); printf("[run VEML6030] header: %s\n", pb);
    BufferPrint bp3(pb, sizeof pb); s.logReading(bp3); s.endReadings(); printf("[run VEML6030] row: %s\n", pb);
    onReading = nullptr; }

  // 9. A dead VEML6030 (no acknowledge on the first reading) stops its batch of 8.
  loadStandard();
  { Libelle s; s.begin(); int k = 0;
    onReading = [&](TwoWire& w) { k++; w.image[0x40] = 0x85; w.image[0x47] = 0x21; };   // chip 1, kind 1
    s.setLightReadings(8); bool ok = s.updateMeasurements(Libelle::VEML6030);
    printf("[dead VEML6030] N=8: update=%d readings taken=%d lightCount=%u lux=%.2f note='%s'\n", ok, k, s.getLightCount(), s.getLux(), s.reportNote().c_str());
    onReading = nullptr; }

  // 10. Cost of one getString() with one reading of everything: bridge transactions plus three accelerometer axes.
  loadStandard();
  { Libelle s; s.begin(); unsigned t0 = Wire.transactions; s.getString(); printf("[cost] requestFrom calls for one getString(): %u\n", Wire.transactions - t0); }

  // 11. The status line for a logger's status file.
  loadStandard();
  { Libelle s; s.begin(); s.updateMeasurements(); char sb[260]; BufferPrint sp(sb, sizeof sb); size_t k = s.printStatus(sp); printf("[status] %zu bytes: %s\n", k, sb); }

  fprintf(stderr, "bus transactions total: %u\n", Wire.transactions);   // metric, not output
  return 0;
}
