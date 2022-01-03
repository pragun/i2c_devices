/*
 * i2c_device.h
 *
 *  Created on: Aug 22, 2021
 *      Author: Pragun
 */

#ifndef SRC_I2C_DEVICE_H_
#define SRC_I2C_DEVICE_H_

#include "i2c.h"
#include "i2c_bus.h"

class I2C_Device{
public:
	I2C_Device(I2C_Bus *i2c_bus, uint8_t address):
		i2c_bus(i2c_bus),
		address(address),
		bus_root(i2c_bus->bus_root())
	{};

	I2C_Status write_n(){
		return I2C_OK;
	}

	I2C_Status read_n(){
		return I2C_OK;
	}

	I2C_Status write_n_then_read_m(uint8_t* tx, uint8_t txn, uint8_t* rx, uint8_t rxn){
		return this->bus_root->write_n_then_read_m(this->address, tx,txn,rx,rxn);
	}

	I2C_Status start_transaction(){
		return this->i2c_bus->start_transaction();
	}

	I2C_Status end_transaction(){
		return this->i2c_bus->end_transaction();
	}

	I2C_Bus_Root* get_bus_root(){
		return this->bus_root;
	}

	I2C_Status read_two_bytes(uint8_t reg, uint8_t* rx_buf){
		return this->write_n_then_read_m(&reg,1,rx_buf,2);
	}

	I2C_Status read_one_byte(uint8_t reg, uint8_t* rx_buf){
		return this->write_n_then_read_m(&reg,1,rx_buf,1);
	}


private:
	I2C_Bus *i2c_bus;
	uint8_t address;
	I2C_Bus_Root *bus_root;
};

#endif /* SRC_I2C_DEVICE_H_ */
