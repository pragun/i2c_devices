/*
 * xCA9584_i2c_bus_mux.h
 *
 *  Created on: Aug 22, 2021
 *      Author: Pragun
 */

#ifndef SRC_XCA9548_I2C_BUS_MUX_H_
#define SRC_XCA9548_I2C_BUS_MUX_H_

#include "i2c.h"
#include "i2c_bus.h"
#include "i2c_device.h"
#include "boost/preprocessor/repetition/repeat.hpp"

class xCA9548_Device;

#define xCA9548_N_Busses 8
#define xCA9548_Addr 0x08

class xCA9548_Bus : public I2C_Bus{
public:
	xCA9548_Bus(uint8_t bus_index, xCA9548_Device *mux_device):
		mux_device(mux_device),
		bus_index(bus_index){
	}

	I2C_Status start_transaction();
	I2C_Status end_transaction();
	I2C_Bus_Root* bus_root();

private:
	xCA9548_Device *mux_device;
	uint8_t bus_index;
};

#define xCA_bus(Z,n,D) xCA9548_Bus(n,this),

class xCA9548_Device : public I2C_Device {
public:
	xCA9548_Device(I2C_Bus* i2c_bus, uint8_t address_modification):
		I2C_Device(i2c_bus, address_modification + xCA9548_Addr),
		//muxed_busses{xCA_bus(0,0,0),xCA_bus(0,0,0)}{}
		muxed_busses{BOOST_PP_REPEAT(xCA9548_N_Busses,xCA_bus,_)}{}


	xCA9548_Bus& operator[](int index);

	I2C_Status start_transaction_on(uint8_t bus_idx);

	I2C_Status end_transaction_on(uint8_t bus_idx);

private:
	xCA9548_Bus muxed_busses[xCA9548_N_Busses];
};

#endif /* SRC_XCA9548_I2C_BUS_MUX_H_ */
