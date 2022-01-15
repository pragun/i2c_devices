#pragma once

#ifndef _HARDWARE_I2C_H
#include "hardware/i2c.h"
#endif

#include "i2c_bus.h"
#include <FreeRTOS.h>
#include <task.h>

class I2C_Interrupt_Master: public I2C_Bus_Root{
    public:
        I2C_Interrupt_Master(i2c_inst_t*, uint);

        uint init();
        int write_n(uint8_t addr, const uint8_t *src, size_t len, bool nostop);
        int read_n(uint8_t addr, uint8_t *dest, size_t len, bool nostop);
        I2C_Status write_n_then_read_m(uint8_t addr, const uint8_t *src, size_t src_len, uint8_t* dest,
        		size_t dest_len, bool without_stop, bool nostop, uint32_t timeout_ticks);
        I2C_Status start_transaction();
        I2C_Status end_transaction();

    private:
        void _irq_handler();
        i2c_inst_t* i2c;
        const TaskHandle_t* task_handle;
        const uint baudrate;
};
