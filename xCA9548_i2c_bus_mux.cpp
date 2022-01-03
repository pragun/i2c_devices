/*
 * xCA_9548_i2c_bus_mux.cpp
 *
 *  Created on: Aug 22, 2021
 *      Author: Pragun
 */

#include "xCA9548_i2c_bus_mux.h"
//#include "printf.h"

xCA9548_Bus& xCA9548_Device::operator[](int index){
	return muxed_busses[index];
}

I2C_Status xCA9548_Device::start_transaction_on(uint8_t bus_idx){
	I2C_Status status = this->start_transaction();
	// set bus state
	//printf("Setting bus state for transaction on Bus:%d, of PCA9548\n\r",bus_idx);
	return status;
}

I2C_Status xCA9548_Device::end_transaction_on(uint8_t bus_idx){
	// reset bus state
	//printf("Re-setting bus state for transaction on Bus:%d, of PCA9548\n\r",bus_idx);
	return this->end_transaction();
}

I2C_Status xCA9548_Bus::start_transaction(){
	return this->mux_device->start_transaction_on(bus_index);
}

I2C_Status xCA9548_Bus::end_transaction(){
	return this->mux_device->end_transaction_on(bus_index);
}

I2C_Bus_Root* xCA9548_Bus::bus_root(){
	return mux_device->get_bus_root();
}

