/*
 * motions.c
 *
 *  Created on: May 12, 2011
 *      Author: dko
 */
#include "global.h"
#include "motor.h"
#include "hardware_def.h"
#include "encoder.h"

#define ROBO_SPEED 70

void inch_right(void) 
{
	set_motor_position_abs(MOTOR_BACK_SIDE, 55, ROBO_SPEED);
	wait_motor(MOTOR_BACK_SIDE);
	set_motor_position_abs(MOTOR_FRONT_SIDE, -55, ROBO_SPEED);
	wait_motor(MOTOR_FRONT_SIDE);
	set_motor_position_abs(MOTOR_BACK_SIDE, 0, ROBO_SPEED);
	wait_motor(MOTOR_BACK_SIDE);
	set_motor_position_abs(MOTOR_FRONT_SIDE, 0, ROBO_SPEED);
	wait_motor(MOTOR_FRONT_SIDE);
}

void inch_left(void)
{
	set_motor_position_abs(MOTOR_FRONT_SIDE, -55, ROBO_SPEED);
	wait_motor(MOTOR_FRONT_SIDE);
	set_motor_position_abs(MOTOR_BACK_SIDE, 55, ROBO_SPEED);
	wait_motor(MOTOR_BACK_SIDE);
	set_motor_position_abs(MOTOR_FRONT_SIDE, 0, ROBO_SPEED);
	wait_motor(MOTOR_FRONT_SIDE);
	set_motor_position_abs(MOTOR_BACK_SIDE, 0, ROBO_SPEED);
	wait_motor(MOTOR_BACK_SIDE);
}

void stand(void)
{
	set_motor_position_abs(MOTOR_FRONT_SIDE, -84, 70);
	wait_motor(MOTOR_FRONT_SIDE);
	set_motor_position_abs(MOTOR_BACK_SIDE, 75, 70);
	wait_motor(MOTOR_BACK_SIDE);
  /* Now in fetal position */
	msleep(500);
  /* Twist bottom plate */
	set_motor_position_abs(MOTOR_FRONT_FRONT, 180, 50);
	wait_motor(MOTOR_FRONT_FRONT);
	msleep(250);
  /* Lift */
	set_motor_position(MOTOR_FRONT_SIDE, 20, 55);
	set_motor_position_abs(MOTOR_BACK_SIDE, 85, 30);
	wait_motor(MOTOR_FRONT_SIDE);
}

void unstand(void)
{
	set_motor_position_abs(MOTOR_FRONT_SIDE, -90, 50);
	set_motor_position_abs(MOTOR_BACK_SIDE, 85, 50);
	wait_motor(MOTOR_BACK_SIDE);
	wait_motor(MOTOR_FRONT_SIDE);
  flat();
}

void roll_forward(void) {
	int32_t enc_ff, enc_bf;
	int i;
	for(i = 0; i < 2; i++) {
		enc_ff = EncoderRead(ENC_FRONT_FRONT);
		enc_bf = EncoderRead(ENC_BACK_FRONT);
		set_motor_position(MOTOR_FRONT_FRONT, enc_ff-180, -ROBO_SPEED);
		set_motor_position(MOTOR_BACK_FRONT, enc_bf-180, -ROBO_SPEED);
		wait_motor(MOTOR_FRONT_FRONT);
		wait_motor(MOTOR_BACK_FRONT);
	}
}

void roll_backward(void) {
	int32_t enc_ff, enc_bf;
	int i;
	for(i = 0; i < 2; i++) {
		enc_ff = EncoderRead(ENC_FRONT_FRONT);
		enc_bf = EncoderRead(ENC_BACK_FRONT);
		set_motor_position(MOTOR_FRONT_FRONT, enc_ff+180, ROBO_SPEED);
		set_motor_position(MOTOR_BACK_FRONT, enc_bf+180, ROBO_SPEED);
		wait_motor(MOTOR_FRONT_FRONT);
		wait_motor(MOTOR_BACK_FRONT);
	}
}

void rotate_left(void)
{
	/* Roll FF backward, BF forward */
	int32_t enc_ff, enc_bf;
	int i;
	for(i = 0; i < 2; i++) {
		enc_ff = EncoderRead(ENC_FRONT_FRONT);
		enc_bf = EncoderRead(ENC_BACK_FRONT);
		set_motor_position(MOTOR_FRONT_FRONT, enc_ff+180, ROBO_SPEED);
		set_motor_position(MOTOR_BACK_FRONT, enc_bf-180, -ROBO_SPEED);
		wait_motor(MOTOR_FRONT_FRONT);
		wait_motor(MOTOR_BACK_FRONT);
	}
}

void rotate_right(void)
{
	/* Roll FF backward, BF forward */
	int32_t enc_ff, enc_bf;
	int i;
	for(i = 0; i < 2; i++) {
		enc_ff = EncoderRead(ENC_FRONT_FRONT);
		enc_bf = EncoderRead(ENC_BACK_FRONT);
		set_motor_position(MOTOR_FRONT_FRONT, enc_ff-180, -ROBO_SPEED);
		set_motor_position(MOTOR_BACK_FRONT, enc_bf+180, ROBO_SPEED);
		wait_motor(MOTOR_FRONT_FRONT);
		wait_motor(MOTOR_BACK_FRONT);
	}
}

void arch(void)
{
	set_motor_position_abs(MOTOR_BACK_SIDE, -50, ROBO_SPEED);
	set_motor_position_abs(MOTOR_FRONT_SIDE, 50, ROBO_SPEED);
	//wait_motor(MOTOR_BACK_SIDE);
	//wait_motor(MOTOR_FRONT_SIDE);
}

void flat(void)
{
	set_motor_position_abs(MOTOR_BACK_SIDE, 0, ROBO_SPEED);
	set_motor_position_abs(MOTOR_FRONT_SIDE, 0, ROBO_SPEED);
	wait_motor(MOTOR_BACK_SIDE);
	wait_motor(MOTOR_FRONT_SIDE);
}

void home(void)
{
	int i;
	for(i = 0; i < 4; i++) {
		set_motor_position_abs(i, 0, 50);
	}
	for(i = 0; i < 4; i++) {
		wait_motor(i);
	}
}
