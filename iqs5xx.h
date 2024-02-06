#pragma once
#include "i2c.h"
#include "i2c_bus.h"
#include "i2c_device.h"


#define CHECK_BIT(a,i) ((a & (0b1 << i)) != 0)
#define READ_CYCLE_TIME //For now this seems important for the IQS5XX to work properly

extern uint32_t time_us_32();

#define CHK(A) if(A != I2C_OK){return A;}

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
    const uint16_t GestureEvt0 = 0x000D;
    const uint16_t GestureEvt1 = 0x000E;
    const uint16_t SysInfo0 = 0x000F;
    const uint16_t SysInfo1 = 0x0010;
    const uint16_t NumFingers = 0x0011;
    const uint16_t Relative_X = 0x0012;
    const uint16_t Relative_Y = 0x0014;
    const uint16_t AbsoluteX_Base = 0x0016;	//(READ) 2 BYTES	//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
    const uint16_t AbsoluteY_Base = 0x0018;	//(READ) 2 BYTES	//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5	 
    const uint16_t TouchStrength_Base = 0x001A;	//(READ) 2 BYTES	//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
    const uint16_t Area_Base = 0x001C;	//(READ)			//ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
    const uint16_t FingerStartingMem[MaxTouches] = {0x0, 0x0007, 0x000E, 0x0015, 0x001C};
    const uint16_t EndCommunication = 0xEEEE;
    const uint16_t SysConfig0 = 0x058E;
    const uint16_t SysConfig1 = 0x058F;
    const uint16_t SingleFingerGestures = 0x06B7;
    const uint16_t MultiFingerGestures = 0x06B8;
    const uint16_t ActiveReportRate = 0x057A;
};

class IQS5XX_Device : public I2C_Device{
public:
	IQS5XX_Device(I2C_Bus* i2c_bus, uint32_t default_timeout_ms):
		I2C_Device(i2c_bus, IQS5XX::address, default_timeout_ms)
		{};

    struct touch_xy_t{
        uint16_t x;
        uint16_t y;
        uint16_t strength;
        uint8_t area;
    };

    uint16_t num_touches;
    touch_xy_t touch_xy[IQS5XX::MaxTouches];
    uint16_t cycle_time = 0;
    uint32_t last_read_time = 0;
    int16_t relative_x;
    int16_t relative_y;
    uint8_t sysinfo0;
    uint8_t sysinfo1;
    uint8_t gesture_evt0;
    uint8_t gesture_evt1;
    uint8_t sys_config0;
    uint8_t sys_config1;
    uint16_t active_report_rate;
    uint8_t single_finger_gestures_config;
    uint8_t multi_finger_gestures_config;
    uint32_t last_x = 0;
    uint32_t last_y = 0;

    inline I2C_Status ReadTwoBytes(uint16_t addr, uint8_t* data){
        I2C_Status drv_status;
        drv_status = this->write_n_then_read_m((uint8_t*) &addr,2,(uint8_t*) data,2,true,true);
        return drv_status;
    };

    inline I2C_Status ReadByte(uint16_t addr, uint8_t* data){
        I2C_Status drv_status;
        drv_status = this->write_n_then_read_m((uint8_t*) &addr,2,(uint8_t*) data,1,true,true);
        return drv_status;
    };

    I2C_Status ReadSingleTouchData(uint8_t i, touch_xy_t& touch_xy){
        uint16_t addr = 0;
        I2C_Status drv_status;
        addr = IQS5XX::FingerStartingMem[i] + IQS5XX::AbsoluteX_Base;
        drv_status = ReadTwoBytes(addr,(uint8_t*) &touch_xy.x);
        CHK(drv_status);

        addr = IQS5XX::FingerStartingMem[i] + IQS5XX::AbsoluteY_Base;
        drv_status = ReadTwoBytes(addr,(uint8_t*) &touch_xy.y);
        CHK(drv_status);

        addr = IQS5XX::FingerStartingMem[i] + IQS5XX::TouchStrength_Base;
        drv_status = ReadTwoBytes(addr,(uint8_t*) &touch_xy.strength);
        CHK(drv_status);

        addr = IQS5XX::FingerStartingMem[i] + IQS5XX::Area_Base;
        drv_status = ReadByte(addr,(uint8_t*) &touch_xy.area);
        return drv_status;
    }

    I2C_Status ReadTouchData(){
        uint8_t end_comm[3] = {0xEE,0xEE,0xEE};
        uint16_t addr = 0;
        I2C_Status drv_status;
        num_touches = 0;
        //this->start_transaction();
        last_read_time = time_us_32();
#ifdef READ_CYCLE_TIME
        drv_status = ReadByte(IQS5XX::PrevCycleTime,(uint8_t*) &cycle_time);
        CHK(drv_status);
#endif
        drv_status = ReadByte(IQS5XX::SysInfo0,(uint8_t*) &sysinfo0);
        CHK(drv_status);

        drv_status = ReadByte(IQS5XX::SysInfo1,(uint8_t*) &sysinfo1);
        CHK(drv_status);

        drv_status = ReadByte(IQS5XX::GestureEvt0,(uint8_t*) &gesture_evt0);
        CHK(drv_status);

        drv_status = ReadByte(IQS5XX::GestureEvt1,(uint8_t*) &gesture_evt1);
        CHK(drv_status);

        drv_status = ReadByte(IQS5XX::Relative_X, (uint8_t*) &relative_x);
        CHK(drv_status);

        drv_status = ReadByte(IQS5XX::Relative_Y, (uint8_t*) &relative_y);
        CHK(drv_status);

        drv_status = ReadByte(IQS5XX::NumFingers,(uint8_t*) &num_touches);
        CHK(drv_status);

        if((num_touches <= 5) && (num_touches >= 1)){
            for(uint8_t i = 0; i<num_touches; i++){
                ReadSingleTouchData(i,touch_xy[i]);
            }
        }

        drv_status = this->write_n_bytes(end_comm,3);
        CHK(drv_status);

        _n_successes ++;
        //this->end_transaction();

        return drv_status;
    };

    /*
    I2C_Status Config(){
        I2C_Status drv_status;

        drv_status = ReadByte(IQS5XX::SingleFingerGestures, &single_finger_gestures_config);
        printf("SingleFingerGestures: %x\n",single_finger_gestures_config);

        drv_status = ReadByte(IQS5XX::MultiFingerGestures, &multi_finger_gestures_config);
        printf("MultiFingerGestures: %x\n",multi_finger_gestures_config);


        sleep_ms(10);
        uint8_t data2[3] = {0x06, 0xB7, 0x00};
        drv_status = this->write_n_bytes(data2,3);

        sleep_ms(1);
        uint8_t data3[3] = {0x06, 0xB8, 0x00};
        drv_status = this->write_n_bytes(data3,3);

        sleep_ms(10);
        drv_status = ReadByte(IQS5XX::SingleFingerGestures, &single_finger_gestures_config);
        printf("SingleFingerGestures: %x\n",single_finger_gestures_config);

        drv_status = ReadByte(IQS5XX::MultiFingerGestures, &multi_finger_gestures_config);
        printf("MultiFingerGestures: %x\n",multi_finger_gestures_config);

        sleep_ms(10);
        drv_status = ReadByte(IQS5XX::SingleFingerGestures, &single_finger_gestures_config);
        printf("SingleFingerGestures: %x\n",single_finger_gestures_config);

        drv_status = ReadByte(IQS5XX::MultiFingerGestures, &multi_finger_gestures_config);
        printf("MultiFingerGestures: %x\n",multi_finger_gestures_config);


        drv_status = ReadByte(IQS5XX::SysConfig0, &sys_config0);
        drv_status = ReadByte(IQS5XX::SysConfig1, &sys_config1);
        printf("SysConfig: %x %x\n",sys_config0, sys_config1);

        sys_config1 = 0x85;
        uint8_t data[3] = {0x05, 0x8F, sys_config1};
        drv_status = this->write_n_bytes(data,3);

        sleep_ms(10);
        drv_status = ReadByte(IQS5XX::SysConfig0, &sys_config0);
        drv_status = ReadByte(IQS5XX::SysConfig1, &sys_config1);
        printf("SysConfig: %x %x\n",sys_config0, sys_config1);

        sleep_ms(10);
        drv_status = ReadByte(IQS5XX::SysConfig0, &sys_config0);
        drv_status = ReadByte(IQS5XX::SysConfig1, &sys_config1);
        printf("SysConfig: %x %x\n",sys_config0, sys_config1);

        drv_status = ReadTwoBytes(IQS5XX::ActiveReportRate, (uint8_t*) &active_report_rate);
        printf("ActiveReportRate: %d\n",active_report_rate);

        return drv_status;
    }; */

private:
};
