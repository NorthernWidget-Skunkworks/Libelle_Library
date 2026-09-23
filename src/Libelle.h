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
#include <NW_Core.h>   // NW_Core: NW_Device (Schema 1 protocol), NW_Readings, NW_Report

/// Lowest firmware patch (Page 0 byte 0x0A) this library accepts: patch 1
/// brought Schema 1 (Page 0, Block 0 handshake, data at 0x48-0x5D).
#define LIBELLE_FW_MIN_PATCH 1

// Readings per updateMeasurements() are kept in static arrays of this
// capacity (one per chip group; no heap); set<Group>Readings(n) clamps to it.
// Override before the include to trade RAM for a longer batch. The default is
// 8, not the 16 of the other libraries: Libelle stores ten fields and a
// logger usually carries two units (UP and DOWN).
#ifndef LIBELLE_UV_CAPACITY
  #define LIBELLE_UV_CAPACITY 8      // VEML6075: UVA, UVB
#endif
#ifndef LIBELLE_LIGHT_CAPACITY
  #define LIBELLE_LIGHT_CAPACITY 8   // VEML6030: ALS, white, lux
#endif
#ifndef LIBELLE_IR_CAPACITY
  #define LIBELLE_IR_CAPACITY 8      // ADS1115: IR short, IR mid, thermistor temperature
#endif
#ifndef LIBELLE_TILT_CAPACITY
  #define LIBELLE_TILT_CAPACITY 8    // ADXL343: roll, pitch
#endif

enum Orientation { UP = 0, DOWN = 1 };

/// Sentinel returned by every getter when it has no reading: the accelerometer
/// is unresponsive (all three axes return identical values), a chip faulted, or
/// the device never answered. The same value as NW_ERROR.
constexpr float LIBELLE_ERROR = NW_ERROR;

/**
 * @class Libelle
 * @brief Library for the Libelle multi-spectral shortwave pyranometer
 * @details Six spectral bands from UV-B through short-wave infrared, housing
 * temperature, and tilt (roll and pitch) from an onboard accelerometer. The
 * ATtiny841 bridge serves the light and infrared sensors as one Schema 1
 * register map; on hardware v1 the ADXL343 accelerometer sits on the
 * controller's I2C bus and the library reads it directly.
 */
class Libelle : public NW_Sensor
{
	public:
		/** @brief Default I2C addresses: NW-Device-Specification Schema 1 'L' (0x4C) facing UP; 'L' XOR 0x40 (0x0C) facing DOWN, by the solder jumper. */
		static constexpr uint8_t DEFAULT_ADDRESS_UP = 0x4C;
		static constexpr uint8_t DEFAULT_ADDRESS_DOWN = 0x0C;
		/** @brief Chip groups a reading can cover (the spec's chip table: 0 VEML6075, 1 VEML6030, 2 ADS1115, 3 ADXL343). */
		enum Component : uint8_t {
			VEML6075 = 0x01,  ///< UVA, UVB
			VEML6030 = 0x02,  ///< ambient light, white, lux
			ADS1115  = 0x04,  ///< IR short, IR mid, thermistor temperature
			ADXL343  = 0x08,  ///< roll, pitch (read on the controller bus on hardware v1)
			BRIDGE   = 0x07,  ///< the three chips behind the ATtiny841
			ALL      = 0x0F
		};
		Libelle(Orientation orientation = UP);
		/**
		 * @brief Begin communications with the Libelle bridge and its accelerometer.
		 * @details Refuses the bridge unless Page 0 says Schema 1, the name
		 * "Libelle", and a firmware patch of at least LIBELLE_FW_MIN_PATCH;
		 * beginFailure() says which gate refused. Takes no reading.
		 * @param[in] ADR_: I2C address of the bridge; 0 (the default) selects
		 * DEFAULT_ADDRESS_UP or DEFAULT_ADDRESS_DOWN by the orientation.
		 * @return True if the bridge passed the three gates and the accelerometer acknowledged.
		 */
		bool begin(uint8_t ADR_ = 0);
		/** @brief Roll [deg] of the last updateMeasurements() (mean over its readings); LIBELLE_ERROR when none. */
		float getRoll(bool update = false);
		/** @brief Pitch [deg]. */
		float getPitch(bool update = false);
		/** @brief UVA, compensated counts (rounded mean); LIBELLE_ERROR when none. */
		long getUVA(bool update = false);
		/** @brief UVB, compensated counts. */
		long getUVB(bool update = false);
		/** @brief Ambient light, raw VEML6030 counts. */
		long getALS(bool update = false);
		/** @brief White (broadband), raw VEML6030 counts. */
		long getWhite(bool update = false);
		/** @brief Illuminance [lx]: ALS x auto-range multiplier x 0.0036, per reading. */
		float getLux(bool update = false);
		/** @brief Near-IR (short) transimpedance output [V]. */
		float getIR_Short(bool update = false);
		/** @brief Near-IR (mid) transimpedance output [V]. */
		float getIR_Mid(bool update = false);
		/** @brief Housing temperature [C] from the thermistor (Steinhart-Hart, per reading). */
		float getTemp(bool update = false);
		/**
		 * @brief Column names with units, each followed by a comma, suffixed _u
		 * (UP) or _d (DOWN): "R_u [deg],P_u [deg],UVA_u,UVB_u,White_u,Vis_u [lx],
		 * IR_S_u,IR_M_u,PyroT_u [C],", with std and sterr columns after a value
		 * when its chip group's statistics are enabled and more than one reading
		 * is configured.
		 */
		String getHeader();
		/** @brief Take a reading of everything (updateMeasurements()) and return the values in getHeader()'s order, LIBELLE_ERROR where a reading failed. */
		String getString();

		/**
		 * @brief Take the configured number of readings of the selected chips
		 * and store them for the getters and the statistics.
		 * @details Each bridge reading triggers the device and waits for its
		 * reading counter to advance (NW-Device-Specification handshake); N > 1
		 * is declared to the device as a batch first. With one reading of all
		 * three bridge chips, they are read in a single transaction. The
		 * accelerometer is read on the controller bus. The single-value getters
		 * return the mean of the readings taken; a chip the device reports
		 * faulted leaves its values at LIBELLE_ERROR.
		 * @param component Libelle::ALL (default) or any OR of the Component bits.
		 * @return true if every selected chip gave at least one valid reading
		 */
		bool updateMeasurements(uint8_t component = ALL);
		/** @brief Take ONE reading of the VEML6075 (UVA, UVB) and append it to the readings. */
		bool updateUV();
		/** @brief Take ONE reading of the VEML6030 (ALS, white, lux) and append it. */
		bool updateLight();
		/** @brief Take ONE reading of the ADS1115 (IR short, IR mid, temperature) and append it. */
		bool updateIR();
		/** @brief Take ONE reading of the ADXL343 (roll, pitch) and append it. */
		bool updateTilt();
		/** @brief Set how many VEML6075 readings updateMeasurements() takes (statistics are computed over them). Clamped to LIBELLE_UV_CAPACITY. @return The number actually set. */
		uint16_t setUVReadings(uint16_t n);
		/** @brief Set how many VEML6030 readings updateMeasurements() takes. Clamped to LIBELLE_LIGHT_CAPACITY. */
		uint16_t setLightReadings(uint16_t n);
		/** @brief Set how many ADS1115 readings updateMeasurements() takes. Clamped to LIBELLE_IR_CAPACITY. */
		uint16_t setIRReadings(uint16_t n);
		/** @brief Set how many ADXL343 readings updateMeasurements() takes. Clamped to LIBELLE_TILT_CAPACITY. */
		uint16_t setTiltReadings(uint16_t n);
		/** @brief Enable or disable the UVA and UVB std and sterr columns in getString()/getHeader(). */
		void setUVStats(bool enable);
		/** @brief Enable or disable the white and lux std and sterr columns. */
		void setLightStats(bool enable);
		/** @brief Enable or disable the IR and temperature std and sterr columns. */
		void setIRStats(bool enable);
		/** @brief Enable or disable the roll and pitch std and sterr columns. */
		void setTiltStats(bool enable);
		/** @brief Number of valid VEML6075 readings stored by the last updateMeasurements(). */
		uint16_t getUVCount();
		/** @brief Number of valid VEML6030 readings stored. */
		uint16_t getLightCount();
		/** @brief Number of valid ADS1115 readings stored. */
		uint16_t getIRCount();
		/** @brief Number of valid ADXL343 readings stored. */
		uint16_t getTiltCount();

		// --- Statistics getters ---
		// Computed two-pass in 32-bit float over the readings stored by the last
		// updateMeasurements() (NW_Readings); LIBELLE_ERROR when none. Lux,
		// temperature, roll and pitch are stored per reading in their physical
		// units, since none is linear in the register values.
		float getUVAMean();      float getUVAStd();      float getUVASterr();      float getUVAMedian();
		float getUVBMean();      float getUVBStd();      float getUVBSterr();      float getUVBMedian();
		float getALSMean();      float getALSStd();      float getALSSterr();      float getALSMedian();
		float getWhiteMean();    float getWhiteStd();    float getWhiteSterr();    float getWhiteMedian();
		float getLuxMean();      float getLuxStd();      float getLuxSterr();      float getLuxMedian();
		float getIR_ShortMean(); float getIR_ShortStd(); float getIR_ShortSterr(); float getIR_ShortMedian();
		float getIR_MidMean();   float getIR_MidStd();   float getIR_MidSterr();   float getIR_MidMedian();
		float getTempMean();     float getTempStd();     float getTempSterr();     float getTempMedian();
		float getRollMean();     float getRollStd();     float getRollSterr();     float getRollMedian();
		float getPitchMean();    float getPitchStd();    float getPitchSterr();    float getPitchMedian();

		// --- Reading interface (NW standard) ---
		/**
		 * @brief Print the header matching printReading(): column names with
		 * units, each followed by a comma, for the chips selected by
		 * beginReadings(). No statistics columns: one reading has none.
		 * @param out Any Print destination (SdFat File, Serial, ...).
		 * @return Bytes written.
		 */
		size_t printHeader(Print& out);
		/**
		 * @brief Print the stored reading of the selected chips, each value
		 * followed by a comma, in getString()'s order. Does not acquire: call
		 * updateMeasurements() first, or use logReading().
		 * @return Bytes written.
		 */
		size_t printReading(Print& out);
		/**
		 * @brief Take ONE reading of the selected chips and print it: the
		 * one-reading primitive for collecting many readings to a file.
		 * @return Bytes written.
		 */
		size_t logReading(Print& out);
		/**
		 * @brief Begin a run of readings, selecting which chips they cover.
		 * @param component Libelle::ALL or any OR of the Component bits.
		 * @param n How many readings the run will take (the number of
		 * logReading() calls to follow); with n > 1 the bridge is told in
		 * advance (readings-requested word).
		 */
		void beginReadings(uint8_t component = ALL, uint16_t n = 0);
		/** @brief End a run of readings. */
		void endReadings();

		// --- Faults (status byte, live; Report register, latched) ---
		/** @brief True if the given chip (0 VEML6075, 1 VEML6030, 2 ADS1115, 3 ADXL343) was faulted in the last reading. The accelerometer fault is the library's own on hardware v1. */
		bool faulted(uint8_t chip);
		/** @brief True if any chip was faulted in the last reading. */
		bool anyFault();
		/** @brief Chip index of the report (7 the unit); meaningful when reportKind() != 0. */
		uint8_t reportChip();
		/** @brief Kind of the report, per the spec's table (1 not answering, 2 timed out, 3 checksum failed, 6 restarted since configured, ...). */
		uint8_t reportKind();
		/** @brief Print the report as text, e.g. "VEML6075: not answering"; "none" when there is no fault. */
		size_t printReport(Print& out);
		/** @brief The report as one word for a note column: "VEML6075NotAnswering", "UnitRestarted"; "UnitNone" when none. */
		String reportNote();
		/** @brief Print one status line for a logger's status file: name, serial, versions, the last report, Pages 0-2 in hex; no newline, not answering. */
		size_t printStatus(Print& out, bool boot = false) override;
		// --- NW_Sensor: the logger's view (Margay::watch) ---
		const char* name() const override { return "Libelle"; }
		bool reportIsFault() override;
		uint8_t bootReportKind() override;
		void clearBootReport() override;
		/** @brief Why the last begin() refused, as one word: "NotAnswering", "NotSchema1", "WrongName", "OldFirmware", "NoAccel"; "None" after success. */
		String beginFailure();
		uint8_t getHardwareMajor();
		uint8_t getHardwareMinor();
		uint8_t getFirmwareVersion();

	private:
		const float LuxRes = 0.0036;

		const float A = 0.003354016;
		const float B = 0.0003074038;
		const float C = 1.019153E-05;
		const float D = 9.093712E-07;

		NW_Device _dev;
		uint8_t Accel_ADR = 0x1D;
		Orientation _orientation = UP;
		bool _bridgeOk = false;   // the bridge passed begin()'s gates
		bool _accelOk = false;    // the accelerometer acknowledged at begin()
		bool _accelFault = false; // the last accelerometer reading failed (not answering or all axes equal)
		// Means of the last updateMeasurements() (or the last logReading()); LIBELLE_ERROR when none.
		float _roll = LIBELLE_ERROR, _pitch = LIBELLE_ERROR;       // [deg]
		long _uva = (long)LIBELLE_ERROR, _uvb = (long)LIBELLE_ERROR; // compensated counts
		long _als = (long)LIBELLE_ERROR, _white = (long)LIBELLE_ERROR; // raw counts
		float _lux = LIBELLE_ERROR;                                  // [lx]
		float _irShort = LIBELLE_ERROR, _irMid = LIBELLE_ERROR;      // [V]
		float _temp = LIBELLE_ERROR;                                 // [C]
		// Readings as the bridge serves them where the scaling is linear (raw
		// counts), else in physical units, one array per field.
		NW_Readings<int32_t, LIBELLE_UV_CAPACITY> _uvaReadings;
		NW_Readings<int32_t, LIBELLE_UV_CAPACITY> _uvbReadings;
		NW_Readings<uint16_t, LIBELLE_LIGHT_CAPACITY> _alsReadings;
		NW_Readings<uint16_t, LIBELLE_LIGHT_CAPACITY> _whiteReadings;
		NW_Readings<float, LIBELLE_LIGHT_CAPACITY> _luxReadings;     // [lx]
		NW_Readings<uint16_t, LIBELLE_IR_CAPACITY> _irShortReadings;
		NW_Readings<uint16_t, LIBELLE_IR_CAPACITY> _irMidReadings;
		NW_Readings<float, LIBELLE_IR_CAPACITY> _tempReadings;       // [C]
		NW_Readings<float, LIBELLE_TILT_CAPACITY> _rollReadings;     // [deg]
		NW_Readings<float, LIBELLE_TILT_CAPACITY> _pitchReadings;    // [deg]
		NW_ReadingsConfig _uvCfg;    // Readings per updateMeasurements() and stats columns, VEML6075 group
		NW_ReadingsConfig _lightCfg; // VEML6030 group
		NW_ReadingsConfig _irCfg;    // ADS1115 group
		NW_ReadingsConfig _tiltCfg;  // ADXL343 group
		uint8_t _component = ALL;    // Selection of the current beginReadings() run
		bool initAccel();
		float getG(uint8_t Axis);
		bool readUV(uint8_t* d);     // Append one served VEML6075 reading (8 bytes from 0x50) unless faulted
		bool readLight(uint8_t* d);  // Append one served VEML6030 reading (6 bytes from 0x48) unless faulted
		bool readIR(uint8_t* d);     // Append one served ADS1115 reading (6 bytes from 0x58) unless faulted
		bool readData();             // One 22-byte read of the three bridge chips, appended
		void resetReadings(uint8_t component);
		void summarise(uint8_t component); // Means into the single-value fields, LIBELLE_ERROR when no reading
		NW_Report report();           // The bridge's report, or the library's own for the accelerometer
		String column(const char* name, const char* unit, bool stats); // "R_u [deg]," plus std and sterr columns
		float TempConvert(float V, float Vcc, float R, float A, float B, float C, float D, float R25);
		void PrintAllRegs();
		bool WriteByte(uint8_t Adr, uint8_t Pos, uint8_t Val);
};

#endif
