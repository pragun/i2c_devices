#include "drv2605.h"

#define WR_Check(R,V) 	status = write_one_byte(R,V); \
						if(status != I2C_OK) { 	\
							this->end_transaction(); \
							return status;} 	\

#define RWR_Check(R,V)  status = read_one_byte(R,&prev_value); \
						WR_Check(R,V) \
						status = read_one_byte(R,&reg_value);


#include <FreeRTOS.h>
#include <task.h>

using namespace DRV2605;

I2C_Status DRV2605_Device::init(float rated_voltage, float overdrive_voltage) {
	I2C_Status status;
	uint8_t reg_value;
	uint8_t prev_value;

	this->start_transaction();

	uint8_t data_buf[35];

	status = read_one_byte(REG_MODE,&prev_value);
	do{
		//RWR_Check(REG_MODE,0x80);  // Device Reset
		vTaskDelay(1);
		WR_Check(REG_MODE,0x80);
		vTaskDelay(1);
		status = read_one_byte(REG_MODE,&reg_value);
	}while(reg_value != 0x40);

	//read_n_bytes(REG_STATUS,data_buf,35);

	RWR_Check(REG_MODE,0x00);  // out of standby

	//uint8_t rv = 255*(rated_voltage/5.36);
	uint8_t rv = 0x90;
	RWR_Check(REG_RATEDV, rv);

	//uint8_t odclamp = 255*(overdrive_voltage/5.6);
	uint8_t odclamp = 0xA4;
	RWR_Check(REG_CLAMPV, odclamp);

	RWR_Check(REG_LIBRARY,0x02);

	RWR_Check(REG_RTPIN,0x00); // no real-time-playback
	RWR_Check(REG_OVERDRIVE, 0); // no overdrive
	RWR_Check(REG_SUSTAINPOS, 0);
	RWR_Check(REG_SUSTAINNEG, 0);
	RWR_Check(REG_BREAK, 0);

	status = read_one_byte(REG_FEEDBACK,&reg_value); // turn off N_ERM_LRA
	WR_Check(REG_FEEDBACK, (reg_value &  0x7F));

	status = read_one_byte(REG_CONTROL3,&reg_value);
	RWR_Check(REG_CONTROL3, (reg_value |  0x20)); // turn on ERM_OPEN_LOOP

	RWR_Check(REG_WAVESEQ1, 1); //
	RWR_Check(REG_WAVESEQ2, 83); // end sequence
	RWR_Check(REG_WAVESEQ3, 70); // end sequence
	RWR_Check(REG_WAVESEQ4, 13); // end sequence
	RWR_Check(REG_WAVESEQ5, 0); // end sequence
	RWR_Check(REG_GO, 1);

	//read_n_bytes(REG_STATUS,data_buf,35);

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

