#include "drv2605.h"

#define WR_Check(R,V) 	status = write_one_byte(R,V); \
						if(status != I2C_OK) { 	\
							this->end_transaction(); \
							return status;} 	\

#include <FreeRTOS.h>
#include <task.h>

using namespace DRV2605;

I2C_Status DRV2605_Device::init() {
	I2C_Status status;
	uint8_t reg_value;
	this->start_transaction();

	WR_Check(REG_MODE,0x00);  // out of standby
	WR_Check(REG_RTPIN,0x00); // no real-time-playback
	WR_Check(REG_WAVESEQ1,0x01); // strong click
	WR_Check(REG_WAVESEQ2, 0); // end sequence
	WR_Check(REG_OVERDRIVE, 0); // no overdrive
	WR_Check(REG_SUSTAINPOS, 0);
	WR_Check(REG_SUSTAINNEG, 0);
	WR_Check(REG_BREAK, 0);

	vTaskDelay( 50 );

	status = this->read_one_byte(REG_FEEDBACK,&reg_value); // turn off N_ERM_LRA
	WR_Check(REG_FEEDBACK, (reg_value &  0x7F));

	status = this->read_one_byte(REG_CONTROL3,&reg_value);
	WR_Check(REG_FEEDBACK, (REG_CONTROL3 |  0x20)); // turn on ERM_OPEN_LOOP

	if (status == I2C_OK){
		initialized = true;
	}

	this->end_transaction();
	return status;
}

I2C_Status DRV2605_Device::select_library(uint8_t lib) {
	I2C_Status status;
	this->start_transaction();
	WR_Check(REG_LIBRARY, lib);
	this->end_transaction();
	return status;
}


I2C_Status DRV2605_Device::go() {
	I2C_Status status;
	this->start_transaction();
	WR_Check(REG_GO, 1);
	this->end_transaction();
	return status;
}

I2C_Status DRV2605_Device::stop() {
	I2C_Status status;
	this->start_transaction();
	WR_Check(REG_GO, 0);
	this->end_transaction();
	return status;
}

I2C_Status DRV2605_Device::set_mode(uint8_t mode) {
	I2C_Status status;
	this->start_transaction();

	WR_Check(REG_MODE, mode);

	this->end_transaction();
	return status;

}

