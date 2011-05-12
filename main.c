#include "global.h"
#include "main.h"
#include "hardware_def.h"
#include "timer.h"
#include "callback.h"
#include "consoleprint.h"
#include "motor.h"
#include "adc.h"
#include "uart.h"
#include "i2c.h"
#include "encoder.h"

#define FS_CENTER 75
#define FF_CENTER 98
#define BS_CENTER 94
#define BF_CENTER 30

char buffer[50];

void msleep(uint32_t milliseconds)
{
	uint32_t current_time;
	current_time = now;
	while(now < (current_time + milliseconds));
}

int main(void)
{
    uint32_t current_time = 0;
    uint32_t cnt = 0;
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

    //I2cInit(SENSOR_BUS);

    // delay 6 sec
    current_time = now;
    while(now < current_time + 6000);

    // center the wheels if they aren't already
    set_motor_position(MOTOR_FRONT_FRONT, FF_CENTER, 70);
    set_motor_position(MOTOR_BACK_FRONT,  BF_CENTER, 72);
    while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);

    // 1 sec delay
    current_time = now;
    while(now < current_time + 1000);

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
    while(now < current_time + 1500);

//    snprintf(buffer, 30, "Roll\n");
//    consoleprint(buffer);
    // They see me rollin', they hatin'
    while (cnt++ < 8)
    {
//        snprintf(buffer, 30, "%d\n",cnt);
//        consoleprint(buffer);
        set_motor_position(MOTOR_FRONT_FRONT,FF_CENTER + 50, 70);
        set_motor_position(MOTOR_BACK_FRONT,BF_CENTER, 72);
        while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
        current_time = now;
        while(now < current_time + 10);
        set_motor_position(MOTOR_FRONT_FRONT,FF_CENTER, 70);
        set_motor_position(MOTOR_BACK_FRONT,BF_CENTER + 50, 72);
        while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
        current_time = now;
        while(now < current_time + 10);
    }
    cnt = 0;

//    snprintf(buffer, 30, "Roll\n");
//    consoleprint(buffer);
    while (cnt++ < 7)
    {
//        snprintf(buffer, 30, "%d\n",cnt);
//        consoleprint(buffer);
        set_motor_position(MOTOR_BACK_FRONT,BF_CENTER, 72);
        while (motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
        current_time = now;
        while(now < current_time + 10);
        set_motor_position(MOTOR_BACK_FRONT,BF_CENTER + 50, 72);
        while (motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
        current_time = now;
        while(now < current_time + 10);
    }
    cnt = 0;
    while (cnt++ < 7)
    {
//        snprintf(buffer, 30, "%d\n",cnt);
//        consoleprint(buffer);
        set_motor_position(MOTOR_FRONT_FRONT,BF_CENTER, -72);
        while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING);
        current_time = now;
        while(now < current_time + 10);
        set_motor_position(MOTOR_FRONT_FRONT,BF_CENTER + 50, -72);
        while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING);
        current_time = now;
        while(now < current_time + 10);
    }
    cnt = 0;
    while (cnt++ < 8)
    {
//        snprintf(buffer, 30, "%d\n",cnt);
//        consoleprint(buffer);
        set_motor_position(MOTOR_FRONT_FRONT,FF_CENTER + 50, 70);
        set_motor_position(MOTOR_BACK_FRONT,BF_CENTER, 72);
        while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
        current_time = now;
        while(now < current_time + 10);
        set_motor_position(MOTOR_FRONT_FRONT,FF_CENTER, 70);
        set_motor_position(MOTOR_BACK_FRONT,BF_CENTER + 50, 72);
        while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
        current_time = now;
        while(now < current_time + 10);
    }
    cnt = 0;
    // center the wheels if they aren't already
    set_motor_position(MOTOR_FRONT_FRONT, FF_CENTER, 70);
    set_motor_position(MOTOR_BACK_FRONT,  BF_CENTER, 72);
    while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
    while(1);

    return 0;
}
