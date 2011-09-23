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
#include "motions.h"

//#define HOME

uint32_t get_ir_sen(void);
char buffer[30];

void msleep(uint32_t milliseconds)
{
	uint32_t current_time;
	current_time = now;
	while(now < (current_time + milliseconds));
}

double g_motorHandlerTimestep;
int main(void)
{
	int i;
	int32_t enc;
	int32_t enc1, enc2;
    uint32_t current_time = 0;
    SystemInit();
    GpioInit();
    //AdcInit();

    TimerInit(0, 1ul * _millisecond);
    CallbackRegister(MotorHandler, 7ul * _millisecond);
    CallbackEnable(MotorHandler);
    g_motorHandlerTimestep = 0.007;
    TimerEnable(0);
    EncoderInit();
    MotorInit();
    MotorStart();

    I2cInit(SENSOR_BUS);
    I2cInit(MODULE_BUS);
    current_time = now;

    msleep(3000);

    // Move the body joints a little
    home();
    //set_motor_position(MOTOR_BACK_SIDE, 255, 0);
    //set_motor_speed(MOTOR_BACK_SIDE, -20);
    msleep(3000);

#ifdef HOME
    while(1);
#endif

#if 0
    set_motor_position(MOTOR_BACK_SIDE, -40, 0);
    set_motor_speed(MOTOR_BACK_SIDE, -30);
    msleep(2000);
    set_motor_speed(MOTOR_BACK_SIDE, 0);
    while(1);
#endif
#ifdef DEMO1
    for(i = 0; i < 4; i++) {
    	inch_right();
    }
    for(i = 0; i < 2; i++) {
    	rotate_left();
    }
    for(i = 0; i < 4; i++) {
    	inch_right();
    }
#endif
#ifdef DEMO2
    /* Twist one end by 45 degrees */
    set_motor_position(MOTOR_FRONT_FRONT, 180, 100);
    wait_motor(MOTOR_FRONT_FRONT);
    msleep(5000);
    roll_forward();
    roll_forward();
    roll_forward();
    arch();
    roll_forward();
    roll_forward();
    roll_forward();
    //set_motor_position_abs(MOTOR_FRONT_SIDE, 0, 80);
    //set_motor_position_abs(MOTOR_BACK_SIDE, 0, 80);
    for(i = 0; i < 3; i++) {
    	roll_forward();
    }

#endif

#ifdef DEMO2A
    /* Twist one end by 45 degrees */
    set_motor_position(MOTOR_FRONT_FRONT, 180, 100);
    arch();
    wait_motor(MOTOR_FRONT_FRONT);
    msleep(5000);
    //set_motor_position_abs(MOTOR_FRONT_SIDE, 0, 80);
    //set_motor_position_abs(MOTOR_BACK_SIDE, 0, 80);
    for(i = 0; i < 8; i++) {
    	roll_forward();
    }

#endif

#ifdef DEMO3

    for(i = 0; i < 2; i++) {
    	inch_left();
    }

    /* Rotate head up 45 deg. */
    set_motor_position_abs(MOTOR_FRONT_SIDE, 45, 80);
    msleep(250);
    /* Rotate back "wheel" 180 degrees */
    enc = EncoderRead(ENC_BACK_FRONT);
    for(i = 0; i < 4; i++) {
    	enc += 180;
    	set_motor_position(MOTOR_BACK_FRONT, enc, 80);
    	wait_motor(MOTOR_BACK_FRONT);
    }
    msleep(500);
    for(i = 0; i < 4; i++) {
    	enc -= 180;
    	set_motor_position(MOTOR_BACK_FRONT, enc, -80);
    	wait_motor(MOTOR_BACK_FRONT);
    }
    flat();
    stand();
    msleep(250);
    enc = 0;
    for(i = 0; i < 2; i++) {
    	set_motor_position(MOTOR_BACK_FRONT, enc, 40);
    	wait_motor(MOTOR_BACK_FRONT);
    	enc += 180;
    }
    msleep(250);
    enc = 0;
    for(i = 0; i < 4; i++) {
      set_motor_position(MOTOR_BACK_FRONT, enc, -40);
      wait_motor(MOTOR_BACK_FRONT);
      enc -= 180;
    }
    msleep(250);
    set_motor_position_abs(MOTOR_FRONT_FRONT, 0, 50);
    set_motor_position_abs(MOTOR_BACK_FRONT, 0, 50);
    wait_motor(MOTOR_FRONT_FRONT);
    wait_motor(MOTOR_BACK_FRONT);
    unstand();
    for(i = 0; i < 2; i++) {
    	inch_left();
    }
#endif


#ifdef DEMO4
    // offset one motor
    enc1 = 0;
    enc2 = 180;
    set_motor_position(MOTOR_FRONT_FRONT, enc1, 70);
    set_motor_position(MOTOR_BACK_FRONT,enc2, 72);
    wait_motor(MOTOR_FRONT_FRONT);
    wait_motor(MOTOR_BACK_FRONT);
    msleep(1000);

    while (1)
    {
    	enc1 += 180;
    	enc2 += 180;
    	if (get_ir_sen())
        {
    		set_motor_position(MOTOR_FRONT_FRONT, enc1, -70);
            set_motor_position(MOTOR_BACK_FRONT,  enc2, -72);
            wait_motor(MOTOR_FRONT_FRONT); wait_motor(MOTOR_BACK_FRONT);
        }
        else
        {
        	set_motor_position(MOTOR_FRONT_FRONT, enc1, 70);
            set_motor_position(MOTOR_BACK_FRONT,  enc2, 72);
            wait_motor(MOTOR_FRONT_FRONT); wait_motor(MOTOR_BACK_FRONT);
        }
    }
#endif

#ifdef DEMO5

  // offset one motor
  enc1 = 0;
  enc2 = 180;
  set_motor_position(MOTOR_FRONT_FRONT, enc1, 70);
  set_motor_position(MOTOR_BACK_FRONT,enc2, 72);
  wait_motor(MOTOR_FRONT_FRONT);
  wait_motor(MOTOR_BACK_FRONT);
  msleep(500);

  for(i = 0; i < 3; i++) {
    enc1 += 180;
    enc2 += 180;
    set_motor_position(MOTOR_FRONT_FRONT, enc1, 70);
    set_motor_position(MOTOR_BACK_FRONT,  enc2, 72);
    wait_motor(MOTOR_FRONT_FRONT); wait_motor(MOTOR_BACK_FRONT);
  }
  enc1 += 180;
  enc2 += 180;
  set_motor_position(MOTOR_FRONT_FRONT, enc1, 70);
  set_motor_position(MOTOR_BACK_FRONT,  enc2, 72);
  set_motor_position_abs(MOTOR_FRONT_SIDE, 80, 70);
  set_motor_position_abs(MOTOR_BACK_SIDE, 85, 70);
  wait_motor(MOTOR_FRONT_FRONT);
  wait_motor(MOTOR_BACK_FRONT);
  for(i = 0; i < 4; i++) 
  {
    enc1 += 180;
    enc2 += 180;
    set_motor_position(MOTOR_FRONT_FRONT, enc1, 70);
    set_motor_position(MOTOR_BACK_FRONT,  enc2, 72);
    wait_motor(MOTOR_FRONT_FRONT); wait_motor(MOTOR_BACK_FRONT);
  }
  //set_motor_position_abs(MOTOR_BACK_SIDE, 45, 70);
  for(i = 0; i < 9; i++) 
  {
    enc1 += 180;
    enc2 -= 180;
    set_motor_position(MOTOR_FRONT_FRONT, enc1, 70);
    set_motor_position(MOTOR_BACK_FRONT,  enc2, -72);
    wait_motor(MOTOR_FRONT_FRONT); 
    wait_motor(MOTOR_BACK_FRONT);
  }
  for(i = 0; i < 9; i++) 
  {
    enc1 += 180;
    enc2 += 180;
    set_motor_position(MOTOR_FRONT_FRONT, enc1, 70);
    set_motor_position(MOTOR_BACK_FRONT,  enc2, 72);
    wait_motor(MOTOR_FRONT_FRONT); 
    wait_motor(MOTOR_BACK_FRONT);
  }

#endif

    //while(1);

    // ofset one side 50% before we start rolling
    set_motor_position(MOTOR_BACK_FRONT, 450, 72);
    while (motor[MOTOR_FRONT_FRONT].state == MOTOR_MOVING && motor[MOTOR_BACK_FRONT].state == MOTOR_MOVING);
    current_time = now;
    while(now < current_time + 1000);

    while (1)
    {
        if (get_ir_sen())
        {
          motor[MOTOR_FRONT_FRONT].direction = 2;
          motor[MOTOR_BACK_FRONT].direction = 2;
          set_motor_speed(MOTOR_FRONT_FRONT, -50);
          set_motor_speed(MOTOR_BACK_FRONT, 50);
          msleep(250);
        }
        else
        {
          motor[MOTOR_FRONT_FRONT].direction = 1;
          motor[MOTOR_BACK_FRONT].direction = 1;
          set_motor_speed(MOTOR_FRONT_FRONT, 50);
          set_motor_speed(MOTOR_BACK_FRONT, -50);
          msleep(250);
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

