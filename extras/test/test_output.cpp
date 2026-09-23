// Output-regression test for Libelle_Library: compiles src/Libelle.cpp against
// the NW_Core stubs and prints getHeader()/getString()/getters for fixed
// register images. Two chips share the bus: the pyranometer bridge and the
// ADXL343 accelerometer, which the library reads directly (hardware v1).
// run.sh diffs the result against baseline.txt.
#include "Arduino.h"
#include "Wire.h"
TwoWire Wire;
#include "../../src/Libelle.cpp"
#include "NW_TestSupport.h"

// The ADXL343: a register file of its own at 0x1D (UP) or 0x53 (DOWN), served by the hooks.
static uint8_t adxl[64];
static uint8_t adxlAddress = 0x1D;
static void setAccel(int16_t x, int16_t y, int16_t z) {   // 0.0039 g per LSB: 256 = 1 g
  adxl[0x32] = x & 0xFF; adxl[0x33] = (x >> 8) & 0xFF;
  adxl[0x34] = y & 0xFF; adxl[0x35] = (y >> 8) & 0xFF;
  adxl[0x36] = z & 0xFF; adxl[0x37] = (z >> 8) & 0xFF;
}
static void installBus(uint8_t bridgeAddress) {
  Wire.deviceAddress = bridgeAddress;
  Wire.isPresent = [](uint8_t a) { return Wire.present && (a == Wire.deviceAddress || a == adxlAddress); };
  Wire.hookOnly  = [](uint8_t a) { return a == adxlAddress; };
  Wire.onRequest = [](TwoWire& w, uint8_t n, std::deque<uint8_t>& out) {
    if (w.address() != adxlAddress) return false;                       // the bridge: the image
    for (uint8_t i = 0; i < n; i++) out.push_back(adxl[(w.pointer() + i) & 0x3F]);
    return true;
  };
}

// Legacy register image (firmware before Schema 1), little-endian.
static void loadLegacyImage(int32_t uva, int32_t uvb, uint16_t als, uint16_t white, uint16_t luxMul,
                            uint16_t irMid, uint16_t irShort, uint16_t therm) {
  uint8_t* r = Wire.image; memset(r, 0, sizeof(Wire.image));
  for (int i = 0; i < 4; i++) { r[0x02 + i] = (uva >> (8 * i)) & 0xFF; r[0x07 + i] = (uvb >> (8 * i)) & 0xFF; }
  r[0x0B] = als & 0xFF;     r[0x0C] = als >> 8;
  r[0x0D] = white & 0xFF;   r[0x0E] = white >> 8;
  r[0x10] = luxMul & 0xFF;  r[0x11] = luxMul >> 8;
  r[0x13] = irMid & 0xFF;   r[0x14] = irMid >> 8;
  r[0x15] = irShort & 0xFF; r[0x16] = irShort >> 8;
  r[0x17] = therm & 0xFF;   r[0x18] = therm >> 8;
}

static void report(const char* name, Libelle& s) {
  printf("[%s]\n", name);
  printf("header: %s\n", s.getHeader().c_str());
  printf("string: %s\n", s.getString().c_str());
  printf("getters: uva=%lu uvb=%lu als=%u white=%u lux=%.4f irS=%.4f irM=%.4f temp=%.4f roll=%.4f pitch=%.4f\n",
         s.getUVA(), s.getUVB(), s.getALS(), s.getWhite(), s.getLux(), s.getIR_Short(), s.getIR_Mid(), s.getTemp(), s.getRoll(), s.getPitch());
}

int main() {
  // 1. UP unit, level: ALS 4000 x mult 8 x 0.0036 = 115.2 lx; IR 1.25 V and 2.5 V; thermistor at 25.00 C
  //    (1.65 V: Rt = R25, so the Steinhart-Hart log term is zero); 1 g on Z.
  installBus(0x40); adxlAddress = 0x1D; setAccel(0, 0, 256);
  loadLegacyImage(12345, 678, 4000, 3000, 8, 20000, 10000, 13200);
  { Libelle s(UP); printf("begin=%d\n", s.begin()); report("UP, level", s); }

  // 2. Tilted: X 100, Y 50, Z 230 LSB.
  setAccel(100, 50, 230);
  { Libelle s(UP); s.begin(); report("UP, tilted", s); }

  // 3. DOWN unit: bridge at 0x41, accelerometer at 0x53; a different set of values.
  installBus(0x41); adxlAddress = 0x53; setAccel(-30, 10, 250);
  loadLegacyImage(1000000, 2000000, 65535, 1, 1, 65535, 0, 2640);   // 0.33 V on the thermistor
  { Libelle s(DOWN); printf("begin=%d\n", s.begin()); report("DOWN", s); }

  // 4. Nothing on the bus.
  installBus(0x40); adxlAddress = 0x1D; Wire.present = false;
  { Libelle s(UP); printf("begin with no sensor: %d\n", s.begin()); report("absent", s); }
  Wire.present = true;

  fprintf(stderr, "bus transactions total: %u\n", Wire.transactions);   // metric, not output
  return 0;
}
