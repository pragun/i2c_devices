/*
 * i2c_bus.h
 *
 *  Created on: Aug 22, 2021
 *      Author: Pragun
 */

#ifndef SRC_I2C_BUS_H_
#define SRC_I2C_BUS_H_

#include "i2c.h"

class I2C_Bus_Root;

class I2C_Bus{
public:
//	I2C_Bus();
	virtual I2C_Status start_transaction() = 0;
	virtual I2C_Status end_transaction() = 0;
	virtual I2C_Bus_Root* bus_root() = 0;
//	~I2C_Bus();
};


//I2C_Bus::I2C_Bus() {}
//I2C_Bus::~I2C_Bus() {}

class I2C_Bus_Root: public I2C_Bus {
public:
	I2C_Bus_Root(I2C_Peripheral *i2c_peripheral): I2C_Bus(),
		i2c_peripheral(i2c_peripheral){
	}

	I2C_Status start_transaction(){
		return i2c_start_transaction(this->i2c_peripheral);
	}

	I2C_Status end_transaction(){
		return i2c_stop_transaction(this->i2c_peripheral);
	}

	I2C_Bus_Root* bus_root(){
		return this;
	}

	I2C_Status write_n_then_read_m(uint8_t address, uint8_t* tx, uint8_t txn, uint8_t* rx, uint8_t rxn){
		//return I2C_OK;
		return i2c_write_n_then_read_m(this->i2c_peripheral, address, tx, txn, rx, rxn);
	}

private:
	I2C_Peripheral *i2c_peripheral;
};

#endif /* SRC_I2C_BUS_H_ */
