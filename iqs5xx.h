
#ifndef __AS5600
#define __AS5600

#include "i2c.h"
#include "i2c_bus.h"
#include "i2c_device.h"


#define CHECK_BIT(a,i) ((a & (0b1 << i)) != 0)


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

    uint8_t num_touches;
    touch_xy_t touch_xy[IQS5XX::MaxTouches];

private:
};

#endif /* SRC_AS5600_H_ */
