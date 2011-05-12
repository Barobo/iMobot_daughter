#include "global.h"
#include "main.h"
#include "hardware_def.h"
#include "timer.h"
#include "callback.h"
#include "consoleprint.h"
#include "encoder.h"
#include "motor.h"
#include "adc.h"
#include "uart.h"
#include "i2c.h"


#define BS_CENTER 81
#define BF_CENTER 75
#define FS_CENTER 77
#define FF_CENTER 10

uint32_t get_ir_sen(void);
char buffer[30];

int main(void)
{
    uint32_t current_time = 0;
    SystemInit();
    GpioInit();
    //AdcInit();

    TimerInit(0, 1ul * _millisecond);
    CallbackRegister(MotorHandler, 10ul * _millisecond);
    CallbackEnable(MotorHandler);
    TimerEnable(0);
    EncoderInit();
    MotorInit();
    MotorStart();

    I2cInit(SENSOR_BUS);
    current_time = now;
    while(now < current_time + 1000);
    // center the wheels if they aren't already
    set_motor_position(MOTOR_FRONT_FRONT, FF_CENTER, 70);
    set_motor_position(MOTOR_BACK_FRONT,  BF_CENTER, 72);
    while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);

    current_time = now;
    while(now < current_time + 5000);

    // offset one motor
    set_motor_position(MOTOR_FRONT_FRONT, FF_CENTER, 70);
    set_motor_position(MOTOR_BACK_FRONT,  BF_CENTER + 50, 72);
    while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
    current_time = now;
    while(now < current_time + 1000);

    while (1)
    {
        if (get_ir_sen())
        {
            set_motor_position(MOTOR_FRONT_FRONT, FF_CENTER + 50, -70);
            set_motor_position(MOTOR_BACK_FRONT,  BF_CENTER, -72);
            while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
        }
        else
        {
            set_motor_position(MOTOR_FRONT_FRONT, FF_CENTER, 70);
            set_motor_position(MOTOR_BACK_FRONT,  BF_CENTER + 50, 72);
            while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);

        }
    }
    return 0;
}

uint32_t get_ir_sen(void)
{
    uint32_t buf[10];
    buf[0] = 0;
    I2cWrite(SENSOR_BUS, 0x90, buf, 2);
    return I2cRead(SENSOR_BUS, 0x90, 1);
}

