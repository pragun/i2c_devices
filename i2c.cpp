
#include "i2c.h"
//#include "printf.h"


I2C_Status i2c_stop_transaction(I2C_Peripheral *a){
	//printf("I2C_Stop_Transaction Independant function.\n\r");
	//i2cReleaseBus(a);
	return I2C_OK;
}

I2C_Status i2c_start_transaction(I2C_Peripheral *a){
	//printf("I2C_Start_Transaction Independant function.\n\r");
	//i2cAcquireBus(a);
	return I2C_OK;
}

I2C_Status i2c_write_n_then_read_m(I2C_Peripheral *a, uint8_t address, uint8_t* tx, uint8_t txn, uint8_t* rx, uint8_t rxn){
	//msg_t status = MSG_OK;
	int status = i2c_write_blocking(a, address, tx, txn, true);
	if (status == PICO_ERROR_GENERIC){
		return I2C_Error;
	}
	status = i2c_read_blocking(a, address, rx, rxn, false);
	if (status == PICO_ERROR_GENERIC){
		return I2C_Error;
	}
		
	return I2C_OK;
}
