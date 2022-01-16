/*
 * as5600.h
 *
 *  Created on: Aug 22, 2021
 *      Author: Pragun
 */

#ifndef SRC_AS5600_H_
#define SRC_AS5600_H_

#include "i2c.h"
#include "i2c_bus.h"
#include "i2c_device.h"


#define CHECK_BIT(a,i) ((a & (0b1 << i)) != 0)


namespace AS5600 {
	constexpr uint8_t address = 0b0110110;
	constexpr uint32_t timeout = 10;
}

class AS5600_Device : public I2C_Device{
public:
	enum Magnet_Status{
			Magnet_OK,
			Magnet_High,
			Magnet_Low,
			No_Magnet,
		};

	enum Magnet_Status_BitPos{
		Magnet_Bit_MH = 3,
		Magnet_Bit_ML = 4,
		Magnet_Bit_MD = 5,
	};

	AS5600_Device(I2C_Bus* i2c_bus, bool flip_direction, uint32_t default_timeout_ms):
		I2C_Device(i2c_bus, AS5600::address, default_timeout_ms),
		initialized(false),
		zero_position(0),
		flip_direction(flip_direction),
		init_magnet_status(No_Magnet),
		_n_erroneous_readings(0),
		max_change(max_change)
		{}

	int16_t read_magnitude();
	int16_t read_raw_angle();
	int16_t read_angle();
	uint8_t read_agc();
	Magnet_Status magnet_status();
	void init_and_zero();

	Magnet_Status get_initial_magnet_status(){
		return this->init_magnet_status;
	}

	uint32_t _n_erroneous_readings = 0;

private:
	bool initialized;
	int16_t zero_position;
	bool flip_direction;
	Magnet_Status init_magnet_status;

	const static uint8_t status_reg = 0x0b;
	const static uint8_t agc_reg = 0x1a;
	const static uint8_t magnitude_reg = 0x1b;

	const static uint8_t raw_angle_reg = 0x0c;
	const static uint8_t angle_reg = 0x0e;

	int16_t last_angle;
	int16_t max_change;
};

#endif /* SRC_AS5600_H_ */
