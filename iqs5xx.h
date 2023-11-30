#pragma once
#include "i2c.h"
#include "i2c_bus.h"
#include "i2c_device.h"


#define CHECK_BIT(a,i) ((a & (0b1 << i)) != 0)
#define READ_CYCLE_TIME //For now this seems important for the IQS5XX to work properly

namespace IQS5XX {
	const uint8_t address = 0b1110100;

    const uint16_t ProductNumber_adr = 0x0000;	//(READ)			//2 BYTES;  
    const uint16_t ProjectNumber_adr = 0x0002;	//(READ)			//2 BYTES;  
    const uint16_t MajorVersion_adr = 0x0004;	//(READ)	
    const uint16_t MinorVersion_adr = 0x0005;	//(READ)	
    const uint16_t BLStatus_adr = 0x0006;
	const uint32_t timeout = 10;
    const uint8_t MaxTouches = 5;
    const uint16_t PrevCycleTime = 0x000C;
    const uint16_t NumFingers = 0x0011;
    //const uint16_t PrevCycleTime = 0x0C00;
    //const uint16_t NumFingers = 0x1100;
    const uint16_t AbsoluteX_Base = 0x0016;	//(READ) 2 BYTES	//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
    const uint16_t AbsoluteY_Base = 0x0018;	//(READ) 2 BYTES	//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5	 
    const uint16_t TouchStrength_Base = 0x001A;	//(READ) 2 BYTES	//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
    const uint16_t Area_Base = 0x001C;	//(READ)			//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
    const uint16_t FingerStartingMem[MaxTouches] = {0x0, 0x0007, 0x000E, 0x0015, 0x001C};
    const uint16_t EndCommunication = 0xEEEE;
};

class IQS5XX_Device : public I2C_Device{
public:
	IQS5XX_Device(I2C_Bus* i2c_bus, uint32_t default_timeout_ms):
		I2C_Device(i2c_bus, IQS5XX::address, default_timeout_ms)
		{};

    struct touch_xy_t{
        uint16_t x;
        uint16_t y;
    };

    uint16_t num_touches;
    touch_xy_t touch_xy[IQS5XX::MaxTouches];
    uint16_t cycle_time = 0;

    I2C_Status ReadTouchData(){
        uint8_t end_comm[3] = {0xEE,0xEE,0xEE};
        uint16_t addr = 0;
        uint16_t naddr = 0;
        I2C_Status drv_status;
        num_touches = 0;
        //this->start_transaction();

#ifdef READ_CYCLE_TIME
        drv_status = this->write_n_then_read_m((uint8_t*) &IQS5XX::PrevCycleTime,2,(uint8_t*) &cycle_time,1,true,true);
#endif
        drv_status = this->write_n_then_read_m((uint8_t*) &IQS5XX::NumFingers,2,(uint8_t*) &num_touches,1,true,true);
        if((num_touches <= 5) && (num_touches >= 1)){
            for(uint8_t i = 0; i<num_touches; i++){
                addr = IQS5XX::FingerStartingMem[i] + IQS5XX::AbsoluteX_Base;
                naddr = (addr & 0xFF00) >> 8;
                naddr |= (addr & 0x00FF) << 8;
                drv_status = this->write_n_then_read_m((uint8_t*) &addr,2,(uint8_t*) &touch_xy[i].x,2,true,true);

                addr = IQS5XX::FingerStartingMem[i] + IQS5XX::AbsoluteY_Base;
                naddr = (addr & 0xFF00) >> 8;
                naddr |= (addr & 0x00FF) << 8;
                drv_status = this->write_n_then_read_m((uint8_t*) &addr,2,(uint8_t*) &touch_xy[i].y,2,true,true);
            }
        }
        drv_status = this->write_n_bytes(end_comm,3);
        //this->end_transaction();

        return drv_status;
    };

private:
};
