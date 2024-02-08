#pragma once
#include "hardware/i2c.h"

#include "i2c.h"
#include "i2c_bus.h"
#include "i2c_device.h"

#include "rp2040_i2c_bus.h"
#include "i2c_interrupt.h"
#include "i2c_bus.h"

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

struct IQS5XXHWConfig{
    uint sda;
    uint scl;
    uint nrst;
    uint rdy;
    uint8_t timeout_ms;
    char name[8];
    uint8_t debug_level;
};

class IQS5XX_Device : public I2C_Device{
private:
    const IQS5XXHWConfig* hw_config_;
    RP2040_I2C_Bus i2c_bus_;

public:
	IQS5XX_Device(const IQS5XXHWConfig* hw_config, I2C_Interrupt_Master* i2c_interrupt1):
        hw_config_(hw_config),
        i2c_bus_(hw_config->scl, hw_config->sda, i2c_interrupt1),
		I2C_Device(IQS5XX::address, (uint32_t) hw_config->timeout_ms)
    {
        this->set_bus(&i2c_bus_);
        gpio_set_function(hw_config_->sda, GPIO_FUNC_I2C);
        gpio_set_function(hw_config_->scl, GPIO_FUNC_I2C);

        gpio_set_function(hw_config_->rdy, GPIO_FUNC_SIO);
        gpio_set_dir(hw_config_->rdy, false);


        gpio_set_function(hw_config_->nrst, GPIO_FUNC_SIO);
        gpio_set_drive_strength(hw_config_->nrst, GPIO_DRIVE_STRENGTH_8MA);
        gpio_set_dir(hw_config_->nrst, true);
        gpio_put(hw_config_->nrst, true);

        gpio_set_irq_enabled(hw_config_->rdy, GPIO_IRQ_EDGE_RISE, true);

    #ifdef USE_INTERRUPTS_FOR_TRACKPAD
        gpio_set_irq_enabled(hw_config_->rdy, GPIO_IRQ_EDGE_RISE, true);
    #endif
                //gpio_set_irq_enabled(hw_config_->key, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);

        bi_decl(bi_2pins_with_func(
                hw_config_->sda,
                hw_config_->scl,
                GPIO_FUNC_I2C));

    };

    struct touch_xy_t{
        uint16_t x;
        uint16_t y;
        uint16_t strength;
        uint8_t area;
    };

    uint16_t num_touches;
    touch_xy_t touch_xy[IQS5XX::MaxTouches];
    uint16_t cycle_time = 0;
    uint32_t last_read_at_time = 0;
    uint32_t time_taken_to_complete_last_read = 0;

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
    char name_[8];

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

    void Reset(){
        gpio_put(hw_config_->nrst, false);
        sleep_ms(1);
        gpio_put(hw_config_->nrst, true);
        sleep_ms(1);
    }

    void debug_touch_read(I2C_Status drv_status){

        if(drv_status == I2C_OK) {
            if(num_touches > 0) {
                int8_t xdiff = (touch_xy[0].x - last_x);
                int8_t ydiff = (touch_xy[0].y - last_y);

                if (hw_config_->debug_level == 1) {
                    printf("%s X:%d Y:%d S:%d A:%d XDiff:%d, YDiff:%d TimeToRead:%d\n",
                           hw_config_->name,
                           touch_xy[0].x, touch_xy[0].y,
                           touch_xy[0].strength, touch_xy[0].area,
                           xdiff, ydiff, time_taken_to_complete_last_read);
                }
                last_x = touch_xy[0].x;
                last_y = touch_xy[0].y;
            }
        }else{
            if(hw_config_->debug_level == 1) {
                printf("Touchpad Read Error\n");
            }
            //leftPaddle_ptr->ResetTouchPad();
        }
    }

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

        if(hw_config_->debug_level == 1){
            debug_touch_read(drv_status);
        }
        return drv_status;
    }

    I2C_Status ReadTouchData(){
        uint8_t end_comm[3] = {0xEE,0xEE,0xEE};
        uint16_t addr = 0;
        I2C_Status drv_status;
        num_touches = 0;
        //this->start_transaction();
        uint32_t t1 = time_us_32();
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
        last_read_at_time = time_us_32();
        time_taken_to_complete_last_read = last_read_at_time - t1;
        return drv_status;
    };

    void print_detailed_debug_output(){
        /* printf("Sys:%x %x Ges:%x %x CTim:%d NTch:%d RX:%d, RY:%d, I2CTim: %dus LastRead: %dus",
             leftTouchpad.sysinfo0,leftTouchpad.sysinfo1,
             leftTouchpad.gesture_evt0,leftTouchpad.gesture_evt1,
             leftTouchpad.cycle_time,
             leftTouchpad.num_touches,
             leftTouchpad.relative_x, leftTouchpad.relative_y,
             i2c_time, time_between_reads);


      if((leftTouchpad.num_touches <= 5) && (leftTouchpad.num_touches >= 1)){
          for(uint8_t i = 0; i<leftTouchpad.num_touches; i++){
              printf("Finger:%d X:%d Y:%d A:%d S:%d \n",
                     i,leftTouchpad.touch_xy[i].x,leftTouchpad.touch_xy[i].y,
                     leftTouchpad.touch_xy[i].area, leftTouchpad.touch_xy[i].strength);
          }
      }*/
    }

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
