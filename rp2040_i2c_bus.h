/*
 * rp2040_i2c_bus.h
 *
 *  Created on: Jan 9, 2022
 *      Author: Pragun
 */

#ifndef I2C_DEVICES_RP2040_I2C_BUS_H_
#define I2C_DEVICES_RP2040_I2C_BUS_H_

/*
 * rp2040_i2c_bus.h
 *
 *  Created on: Aug 22, 2021
 *      Author: Pragun
 */

#include "i2c.h"
#include "i2c_bus.h"
#include "i2c_interrupt.h"
#include "hardware/gpio.h"

class RP2040_I2C_Bus: public I2C_Bus{
public:
	RP2040_I2C_Bus(uint scl_pin,uint sda_pin,I2C_Bus_Root* bus_root):
		scl_pin(scl_pin),
		sda_pin(sda_pin),
		_bus_root(bus_root){
	};

	I2C_Status start_transaction(){
		gpio_set_function(this->scl_pin, GPIO_FUNC_I2C);
		gpio_set_function(this->sda_pin, GPIO_FUNC_I2C);
		return I2C_Status::I2C_OK;
	};

	I2C_Status end_transaction(){
		gpio_set_function(this->scl_pin, GPIO_FUNC_SIO);
		gpio_set_function(this->sda_pin, GPIO_FUNC_SIO);
		return I2C_Status::I2C_OK;
	};

	I2C_Bus_Root* bus_root(){
		return _bus_root;
	};

private:
	uint scl_pin;
	uint sda_pin;
	I2C_Bus_Root* _bus_root;
};


#endif /* I2C_DEVICES_RP2040_I2C_BUS_H_ */
