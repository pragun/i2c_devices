/*
 * as5600.cpp
 *
 *  Created on: Aug 22, 2021
 *      Author: Pragun
 */

#include "as5600.h"
//#include "printf.h"

inline int16_t sign_extend_12bits(int16_t i){
	int m = 1U << 11;
	return ((i ^ m) - m);
}

int16_t AS5600_Device::read_angle(){
	this->start_transaction();
	uint8_t rx_bytes[2] = {0x00, 0x00};

	I2C_Status status = this->read_two_bytes(AS5600_Device::angle_reg,rx_bytes);
	this->end_transaction();

	int16_t raw_angle = rx_bytes[1];
	raw_angle |= rx_bytes[0] << 8;
	int16_t angle = sign_extend_12bits(raw_angle);

	if(initialized){
		if(this->flip_direction)
			angle = this->zero_position - angle;
		else
			angle = angle - this->zero_position;
	}

	//printf("Read AS5600 Read Angle. Retval:%d, RX:0x%04x, Angle:%d\r\n",status,raw_angle, angle);
	return angle;
}

AS5600_Device::Magnet_Status AS5600_Device::magnet_status(){
	this->start_transaction();
	uint8_t rx_byte = 0x00;

	I2C_Status status = this->read_one_byte(AS5600_Device::status_reg,&rx_byte);
	this->end_transaction();

	if (CHECK_BIT(rx_byte,AS5600_Device::Magnet_Bit_MD)){
		if(CHECK_BIT(rx_byte,AS5600_Device::Magnet_Bit_MH)){
			return AS5600_Device::Magnet_High;
		}
		if(CHECK_BIT(rx_byte,AS5600_Device::Magnet_Bit_ML)){
			return AS5600_Device::Magnet_Low;
		}
		return AS5600_Device::Magnet_OK;
	}
	return AS5600_Device::No_Magnet;
}

/* AS5600_Device::Magnet_Status AS5600_Device::get_initial_magnet_status(){
	return this->init_magnet_status;
}*/

void AS5600_Device::init_and_zero(){
	AS5600_Device::Magnet_Status m_status = this->magnet_status();

	this->init_magnet_status = m_status;

	this->zero_position = this->read_angle();

	this->initialized = true;
}
