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

The shortwave module aggregates its sensors through an ATtiny841 acting as an I2C bridge. The host microcontroller communicates with a single I2C address (0x40 facing up, 0x41 facing down); the bridge handles all internal sensor communication.

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
| `begin()` | `bool` | Initialize sensor and I2C bus; returns true on success, false if either the sensor bridge or accelerometer is unreachable; includes 2 ms settling time for accelerometer startup |
| `getHeader()` | `String` | Comma-separated column names with units |
| `getString()` | `String` | Comma-separated measurement values |
| `getUVA()` | `unsigned long` | Raw UV-A counts |
| `getUVB()` | `unsigned long` | Raw UV-B counts |
| `getALS()` | `unsigned int` | Raw ambient light sensor counts |
| `getWhite()` | `unsigned int` | Raw broadband counts |
| `getLux()` | `float` | Illuminance (lux) |
| `getIR_Short()` | `float` | VEMD1060X01 (~700–1100 nm) transimpedance output voltage (V); 47 kΩ feedback resistor, ADS1115 at ±4.096 V FSR |
| `getIR_Mid()` | `float` | SD003-151-001 (~1000–1700 nm) transimpedance output voltage (V); 1 MΩ feedback resistor, ADS1115 at ±4.096 V FSR |
| `getTemp()` | `float` | Housing temperature (°C) |
| `getRoll()` | `float` | Roll angle from accelerometer (degrees); uses all three axes; returns `LIBELLE_ERROR` (-9999) if the accelerometer is unresponsive |
| `getPitch()` | `float` | Pitch angle from accelerometer (degrees); uses all three axes; ADXL343 outputs at 100 Hz — readings taken less than 10 ms apart return the same sample; returns `LIBELLE_ERROR` (-9999) if the accelerometer is unresponsive |

### IR channel units and calibration

`getIR_Short()` and `getIR_Mid()` return the raw output voltage of the transimpedance amplifier circuit (photodiode current × feedback resistance), as read by the ADS1115 ADC. Converting to irradiance (W/m²) requires calibration against a reference pyranometer — a fixed conversion factor is not physically valid because the responsivity of each photodiode varies across its spectral range, so the effective sensitivity changes with the spectral composition of the incoming light (e.g., clear sky vs. overcast).

## Name history

This library has been through two prior names. It began as **Dyson** — a reference to Freeman Dyson and the idea of a structure that follows the sun — and was subsequently renamed **Monarch** after the butterfly. *Libelle* is the third and current name, chosen to avoid unintended connotations while staying within the project's tradition of naming instruments after organisms with exceptional sensory capabilities.

The hardware project lives at [NorthernWidget-Skunkworks/Project-Libelle](https://github.com/NorthernWidget-Skunkworks/Project-Libelle).

## License

Distributed as-is; no warranty is given.

**Full API reference:** https://docs.northernwidget.com/Libelle_Library/
