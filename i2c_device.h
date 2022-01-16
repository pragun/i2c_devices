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
	I2C_Device(I2C_Bus *i2c_bus, uint8_t address, uint32_t default_timeout):
		i2c_bus(i2c_bus),
		address(address),
		bus_root(i2c_bus->bus_root()),
		default_timeout(default_timeout)
	{};

	I2C_Status write_n(){
		return I2C_OK;
	}

	I2C_Status read_n(){
		return I2C_OK;
	}

	I2C_Status write_n_then_read_m(uint8_t* tx, uint8_t txn, uint8_t* rx, uint8_t rxn){
		I2C_Status _status = this->bus_root->write_n_then_read_m(this->address,tx,txn,rx,rxn,true,false,default_timeout);

		switch(_status){
			case I2C_Error:
				_n_errors ++;
				break;

			case I2C_Timed_Out:
				_n_timeouts ++;
				break;

			case I2C_TX_Abort:
				_n_aborts ++;
				break;

			default:
				break;
		}

		return _status;
	}

	I2C_Status start_transaction(){
		transaction_started = true;
		return this->i2c_bus->start_transaction();
	}

	I2C_Status end_transaction(){
		if (transaction_started){
			_n_transactions ++;
		}
		transaction_started = false;
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

	I2C_Status write_one_byte(uint8_t reg, const uint8_t tx){
		uint8_t tx_bytes[2] = {reg, tx};
		return this->write_n_then_read_m(tx_bytes,2,tx_bytes,0);
	}


	uint32_t _n_errors = 0;
	uint32_t _n_aborts = 0;
	uint32_t _n_timeouts = 0;
	uint32_t _n_transactions = 0;


private:
	uint32_t default_timeout;
	I2C_Bus *i2c_bus;
	uint8_t address;
	I2C_Bus_Root *bus_root;
	bool transaction_started = false;
};

#endif /* SRC_I2C_DEVICE_H_ */
