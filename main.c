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

void msleep(uint32_t milliseconds)
{
	uint32_t current_time;
	current_time = now;
	while(now < (current_time + milliseconds));
}

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

    // Move the body joints a little
    set_motor_position(MOTOR_FRONT_SIDE, 255, 0);
    set_motor_position(MOTOR_BACK_SIDE, 255, 0);
    /*
    set_motor_speed(MOTOR_FRONT_SIDE, 30);
    msleep(500);
    set_motor_speed(MOTOR_FRONT_SIDE, 0);
    msleep(1000);
while(1);
*/
    set_motor_speed(MOTOR_BACK_SIDE, -30);
    msleep(500);
    set_motor_speed(MOTOR_BACK_SIDE, 0);
    msleep(2000);

    while(1);

    // ofset one side 50% before we start rolling
    set_motor_position(MOTOR_BACK_FRONT,BF_CENTER + 50, 72);
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

