
#ifndef __DRV2605
#define __DRV2605

#include "i2c.h"
#include "i2c_bus.h"
#include "i2c_device.h"


namespace DRV2605{
	const uint8_t address = 0x5A; ///< Device I2C address

	constexpr uint8_t REG_STATUS = 0x00;       ///< Status register
	constexpr uint8_t REG_MODE = 0x01;         ///< Mode register

	constexpr uint8_t MODE_INTTRIG = 0x00;     ///< Internal trigger mode
	constexpr uint8_t MODE_EXTTRIGEDGE = 0x01; ///< External edge trigger mode
	constexpr uint8_t MODE_EXTTRIGLVL = 0x02;  ///< External level trigger mode
	constexpr uint8_t MODE_PWMANALOG = 0x03;   ///< PWM/Analog input mode
	constexpr uint8_t MODE_AUDIOVIBE = 0x04;   ///< Audio-to-vibe mode
	constexpr uint8_t MODE_REALTIME = 0x05;    ///< Real-time playback (RTP) mode
	constexpr uint8_t MODE_DIAGNOS = 0x06;     ///< Diagnostics mode
	constexpr uint8_t MODE_AUTOCAL = 0x07;     ///< Auto calibration mode

	constexpr uint8_t REG_RTPIN = 0x02;    ///< Real-time playback input register
	constexpr uint8_t REG_LIBRARY = 0x03;  ///< Waveform library selection register
	constexpr uint8_t REG_WAVESEQ1 = 0x04; ///< Waveform sequence register 1
	constexpr uint8_t REG_WAVESEQ2 = 0x05; ///< Waveform sequence register 2
	constexpr uint8_t REG_WAVESEQ3 = 0x06; ///< Waveform sequence register 3
	constexpr uint8_t REG_WAVESEQ4 = 0x07; ///< Waveform sequence register 4
	constexpr uint8_t REG_WAVESEQ5 = 0x08; ///< Waveform sequence register 5
	constexpr uint8_t REG_WAVESEQ6 = 0x09; ///< Waveform sequence register 6
	constexpr uint8_t REG_WAVESEQ7 = 0x0A; ///< Waveform sequence register 7
	constexpr uint8_t REG_WAVESEQ8 = 0x0B; ///< Waveform sequence register 8

	constexpr uint8_t REG_GO = 0x0C;         ///< Go register
	constexpr uint8_t REG_OVERDRIVE = 0x0D;  ///< Overdrive time offset register
	constexpr uint8_t REG_SUSTAINPOS = 0x0E; ///< Sustain time offset, positive register
	constexpr uint8_t REG_SUSTAINNEG = 0x0F; ///< Sustain time offset, negative register
	constexpr uint8_t REG_BREAK = 0x10;      ///< Brake time offset register
	constexpr uint8_t REG_AUDIOCTRL = 0x11;  ///< Audio-to-vibe control register
	constexpr uint8_t REG_AUDIOLVL = 0x12; ///< Audio-to-vibe minimum input level register
	constexpr uint8_t REG_AUDIOMAX = 0x13; ///< Audio-to-vibe maximum input level register
	constexpr uint8_t REG_AUDIOOUTMIN = 0x14; ///< Audio-to-vibe minimum output drive register
	constexpr uint8_t REG_AUDIOOUTMAX = 0x15; ///< Audio-to-vibe maximum output drive register
	constexpr uint8_t REG_RATEDV = 0x16; ///< Rated voltage register
	constexpr uint8_t REG_CLAMPV = 0x17; ///< Overdrive clamp voltage register
	constexpr uint8_t REG_AUTOCALCOMP = 0x18; ///< Auto-calibration compensation result register
	constexpr uint8_t REG_AUTOCALEMP = 0x19;    ///< Auto-calibration back-EMF result register
	constexpr uint8_t REG_FEEDBACK = 0x1A; ///< Feedback control register
	constexpr uint8_t REG_CONTROL1 = 0x1B; ///< Control1 Register
	constexpr uint8_t REG_CONTROL2 = 0x1C; ///< Control2 Register
	constexpr uint8_t REG_CONTROL3 = 0x1D; ///< Control3 Register
	constexpr uint8_t REG_CONTROL4 = 0x1E; ///< Control4 Register
	constexpr uint8_t REG_VBAT = 0x21;     ///< Vbat voltage-monitor register
	constexpr uint8_t REG_LRARESON = 0x22; ///< LRA resonance-period register
};

class DRV2605_Device : public I2C_Device{
public:
	enum ActuatorType{
		ERM,
		LRA
	};

	DRV2605_Device(I2C_Bus* i2c_bus, uint32_t default_timeout_ms):
		I2C_Device(i2c_bus, DRV2605::address, default_timeout_ms),
		initialized(false)
		{}

	I2C_Status init(float rv, float od_v);
	I2C_Status set_mode(uint8_t mode);
	I2C_Status set_waveform(uint8_t slot, uint8_t w);
	I2C_Status select_library(uint8_t lib);
	I2C_Status go();
	I2C_Status stop();
	I2C_Status set_overdrive_voltage();
	I2C_Status set_rated_voltage();

private:
	//I2C_Status set_actuator_type(ActuatorType ac_type);
	//ActuatorType ac_type;
	bool initialized = false;
};

#endif
