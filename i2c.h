/*
 * i2c.h
 *
 *  Created on: Aug 22, 2021
 *      Author: Pragun
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

//#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

using I2C_Peripheral = i2c_inst_t;

enum I2C_Status {
	I2C_OK,
	I2C_Error,
};

I2C_Status i2c_stop_transaction(I2C_Peripheral *a);

I2C_Status i2c_start_transaction(I2C_Peripheral *a);

I2C_Status i2c_write_n_then_read_m(I2C_Peripheral *a, uint8_t address, uint8_t* tx, uint8_t txn, uint8_t* rx, uint8_t rxn);

#endif /* SRC_I2C_H_ */
