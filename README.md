[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4572352.svg)](https://doi.org/10.5281/zenodo.4572352)

# Libelle

**Arduino library for the NorthernWidget shortwave pyranometer**

*Libelle* is the German word for dragonfly — dragonflies are among the most spectrally sophisticated flyers, with up to 30 types of photoreceptors spanning ultraviolet to near-infrared. It also carries a second meaning: *Libelle* is the German word for a spirit level, a nod to the onboard accelerometer used to correct measurements for sensor tilt.

This library interfaces with the NorthernWidget shortwave pyranometer module, which measures solar radiation across six spectral bands from UV-B through short-wave infrared (~280–1700 nm), along with sensor orientation and housing temperature.

The longwave (thermal IR) sensor that was formerly bundled in this repository has been split into a separate companion library: **[Liasis](https://github.com/NorthernWidget-Skunkworks/Liasis_Library)** — named after the genus of Australian water and olive pythons, which detect thermal infrared radiation through labial pit organs sensitive to ~5–30 µm.

## Spectral channels

| Channel | Sensor | Approximate range |
|---------|--------|------------------|
| UV-B | VEML6075 | ~280–315 nm |
| UV-A | VEML6075 | ~315–400 nm |
| Visible (ALS) | VEML6030 | ~400–700 nm |
| White (broadband) | VEML6030 | ~300–700 nm |
| Near-IR (short) | VEMD1060X01 | ~700–1100 nm |
| Near-IR (mid) | SD003-151-001 | ~1000–1700 nm |

In addition, the board reports:

- Tilt (roll and pitch) via an ADXL343 MEMS accelerometer
- Housing temperature via an NTC thermistor

## Hardware

The shortwave module aggregates its sensors through an ATtiny841 acting as an I2C bridge. The host microcontroller communicates with a single I2C address (0x4C facing up, 0x0C facing down, per the [NW-Device-Specification](https://github.com/NorthernWidget/NW-Device-Specification) Schema 1 registry; firmware before Schema 1 answered at 0x40 and 0x41); the bridge handles all internal sensor communication. On hardware v1 the ADXL343 accelerometer sits on the host's I2C bus (0x1D facing up, 0x53 facing down) and the library reads it directly.

Two modules can be stacked on the same I2C bus using the orientation flag — one facing skyward and one facing the ground — for net shortwave radiation measurements.

**Key ICs:** VEML6075 (UV), VEML6030 (visible/lux), VEMD1060X01 (near-IR), SD003-151-001 (short-wave IR), ADXL343 (accelerometer), ADS1115 (ADC), ATtiny841 (I2C bridge)

## Basic usage

```cpp
#include <Libelle.h>

Libelle pyroUp(UP);    // upward-facing module
Libelle pyroDown(DOWN); // downward-facing module

void setup() {
    Serial.begin(38400);
    pyroUp.begin();
    pyroDown.begin();
    Serial.println(pyroUp.getHeader() + pyroDown.getHeader());
}

void loop() {
    Serial.println(pyroUp.getString() + pyroDown.getString());
    delay(1000);
}
```

## API

| Method | Returns | Description |
|--------|---------|-------------|
| `begin(address = 0)` | `bool` | Initialize sensor and I2C bus; returns true on success, false if the bridge is not a Schema 1 Libelle at firmware patch `LIBELLE_FW_MIN_PATCH` or later, or the accelerometer is unreachable (`beginFailure()` says which); includes 2 ms settling time for accelerometer startup; 0 selects the address by orientation |
| `getHeader()` | `String` | Comma-separated column names with units |
| `getString()` | `String` | Takes a reading of everything (`updateMeasurements()`) and returns the comma-separated measurement values; `-9999` where a reading failed |
| `getUVA()` | `long` | UV-A compensated counts |
| `getUVB()` | `long` | UV-B compensated counts |
| `getALS()` | `long` | Raw ambient light sensor counts |
| `getWhite()` | `long` | Raw broadband counts |
| `getLux()` | `float` | Illuminance (lux) |
| `getIR_Short()` | `float` | VEMD1060X01 (~700–1100 nm) transimpedance output voltage (V); 47 kΩ feedback resistor, ADS1115 at ±4.096 V FSR |
| `getIR_Mid()` | `float` | SD003-151-001 (~1000–1700 nm) transimpedance output voltage (V); 1 MΩ feedback resistor, ADS1115 at ±4.096 V FSR |
| `getTemp()` | `float` | Housing temperature (°C) |
| `getRoll()` | `float` | Roll angle from accelerometer (degrees); uses all three axes; returns `LIBELLE_ERROR` (-9999) if the accelerometer is unresponsive |
| `getPitch()` | `float` | Pitch angle from accelerometer (degrees); uses all three axes; ADXL343 outputs at 100 Hz — readings taken less than 10 ms apart return the same sample; returns `LIBELLE_ERROR` (-9999) if the accelerometer is unresponsive |

The value getters return the stored reading, the mean over the readings of the last `updateMeasurements()`; pass `true` to take a fresh one first. Every getter returns `LIBELLE_ERROR` (-9999) when it has no reading: the chip faulted, the device never answered, or the accelerometer was unresponsive.

### Readings, statistics, and faults

`setUVReadings(n)`, `setLightReadings(n)`, `setIRReadings(n)` and `setTiltReadings(n)` set how many readings of the VEML6075 (UVA, UVB), the VEML6030 (ALS, white, lux), the ADS1115 (IR short, IR mid, temperature) and the ADXL343 (roll, pitch) each `updateMeasurements()` takes (clamped to `LIBELLE_UV_CAPACITY` and its siblings, default 8; override before the include). The values printed are then the means, and `getUVAMean()`, `getUVAStd()`, `getUVASterr()`, `getUVAMedian()`, the same for `UVB`, `ALS`, `White`, `Lux`, `IR_Short`, `IR_Mid`, `Temp`, `Roll` and `Pitch`, plus `getUVCount()`, `getLightCount()`, `getIRCount()` and `getTiltCount()`, read the stored readings. With `setUVStats(true)` and its siblings the std and sterr columns join `getHeader()` and `getString()`. `updateMeasurements(Libelle::VEML6030)`, or any OR of the `Component` bits, reads part of the board. For one row per reading to a file, `beginReadings(component, n)`, `printHeader(out)`, `logReading(out)` n times, `endReadings()`, to any `Print` (an SdFat `File`, `Serial`).

Faults, for sketches that want them: `faulted(chip)` (0 VEML6075, 1 VEML6030, 2 ADS1115, 3 ADXL343), `anyFault()`, `faultChip()`, `faultKind()`, `printFault(Serial)`, `faultNote()` (one word, e.g. `VEML6075NoACK`, for a logger's note column). The bridge latches the faults of its three chips; the accelerometer's is the library's own on hardware v1. `begin()` refuses a device that is not Schema 1, not a Libelle, or below firmware patch `LIBELLE_FW_MIN_PATCH`; `beginFailure()` says which (`NoACK`, `NotSchema1`, `WrongName`, `OldFirmware`, `NoAccel`). Requires the [NW_Core](https://github.com/NorthernWidget/NW_Core) library and Libelle firmware patch 1 or later; firmware before Schema 1 is not supported by this version.

### IR channel units and calibration

`getIR_Short()` and `getIR_Mid()` return the raw output voltage of the transimpedance amplifier circuit (photodiode current × feedback resistance), as read by the ADS1115 ADC. Converting to irradiance (W/m²) requires calibration against a reference pyranometer — a fixed conversion factor is not physically valid because the responsivity of each photodiode varies across its spectral range, so the effective sensitivity changes with the spectral composition of the incoming light (e.g., clear sky vs. overcast).

## Name history

This library has been through two prior names. It began as **Dyson** — a reference to Freeman Dyson and the idea of a structure that follows the sun — and was subsequently renamed **Monarch** after the butterfly. *Libelle* is the third and current name, chosen to avoid unintended connotations while staying within the project's tradition of naming instruments after organisms with exceptional sensory capabilities.

The hardware project lives at [NorthernWidget-Skunkworks/Project-Libelle](https://github.com/NorthernWidget-Skunkworks/Project-Libelle).

## License

Distributed as-is; no warranty is given.

**Full API reference:** https://docs.northernwidget.com/Libelle_Library/
