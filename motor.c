#include "global.h"
#include "motor.h"
#include "encoder.h"
#include "consoleprint.h"

motor_t motor[4];
int32_t g_enc[4];

#define MOTOR_CONTROL_TOLERANCE 100
#define MOTOR_GOAL_TOLERANCE 10

uint32_t MotorInit()
{
    int i;
    LPC_PINCON->PINSEL4 = 0x00001555; /* set GPIOs for all PWM pins on PWM0 */

    LPC_PWM1->TCR = TCR_RESET; /* Counter Reset */
    LPC_PWM1->PR = 0x00; /* count frequency:Fpclk */
    LPC_PWM1->MCR = PWMMR0I; /* interrupt on PWMMR0, reset on PWMMR0, reset TC if PWM0 matches */

    LPC_PWM1->MR0 = 10000;
    LPC_PWM1->MR3 = 0;
    LPC_PWM1->MR4 = 0;
    LPC_PWM1->MR5 = 0;
    LPC_PWM1->MR6 = 0;

    /* all PWM latch enabled */
    LPC_PWM1->LER = LER0_EN | LER3_EN | LER4_EN | LER5_EN | LER6_EN;

    //LPC_GPIO0->FIOMASK |= _BIT(16);

    set_gpio_select(M1_PWM, 1);
    set_gpio_select(M1_DIR1, 0);
    set_gpio_select(M1_DIR2, 0);

    set_gpio_select(M2_PWM, 1);
    set_gpio_select(M2_DIR1, 0);
    set_gpio_select(M2_DIR2, 0);

    set_gpio_select(M3_PWM, 1);
    set_gpio_select(M3_DIR1, 0);
    set_gpio_select(M3_DIR2, 0);

    set_gpio_select(M4_PWM, 1);
    set_gpio_select(M4_DIR1, 0);
    set_gpio_select(M4_DIR2, 0);

    set_gpio_dir(M1_DIR1, 1);
    set_gpio_dir(M1_DIR2, 1);
    set_gpio_dir(M2_DIR1, 1);
    set_gpio_dir(M2_DIR2, 1);
    set_gpio_dir(M3_DIR1, 1);
    set_gpio_dir(M3_DIR2, 1);
    set_gpio_dir(M4_DIR1, 1);
    set_gpio_dir(M4_DIR2, 1);

    set_gpio_pin(M1_DIR1, 0);
    set_gpio_pin(M1_DIR2, 0);

    //NVIC_EnableIRQ(PWM1_IRQn);
    for(i = 0; i < 4; i++) {
        motor[i].state = 0;
        motor[i].desired_position = 0;
        motor[i].direction = 0;
        motor[i].speed = 30;
        motor[i].pid_flag = 0;
        motor[i].pid_integ = 0;
        motor[i].pid_integ_gain = 5;
        motor[i].rotation = 0;
    }
    return 1;
}

#define MOTOR_SCALE 100
void set_motor_speed(uint32_t channel, int cycle)
{
    if(cycle != 0) {
        motor[channel].state = MOTOR_MOVING;
    } else {
        motor[channel].state = MOTOR_IDLE;
    }
    /* The direction for the back-side motor has been flipflopped. */
    if(channel == MOTOR_BACK_SIDE) {
      cycle = -cycle;
    }
    if(channel == MOTOR_BACK_FRONT) {
      cycle = -cycle;
    }
    switch(channel)
    {
        case MOTOR_BACK_FRONT:
            if(cycle > 0)
            {
                set_gpio_pin(M1_DIR1, GPIO_OFF);
                set_gpio_pin(M1_DIR2, GPIO_ON);
                LPC_PWM1->MR4 = cycle * MOTOR_SCALE;
            }
            else
            {
                set_gpio_pin(M1_DIR1, GPIO_ON);
                set_gpio_pin(M1_DIR2, GPIO_OFF);
                LPC_PWM1->MR4 = -cycle * MOTOR_SCALE;
            }
            break;
        case MOTOR_BACK_SIDE:
            if(cycle > 0)
            {
                set_gpio_pin(M2_DIR1, GPIO_OFF);
                set_gpio_pin(M2_DIR2, GPIO_ON);
                LPC_PWM1->MR3 = cycle * MOTOR_SCALE;
            }
            else
            {
                set_gpio_pin(M2_DIR1, GPIO_ON);
                set_gpio_pin(M2_DIR2, GPIO_OFF);
                LPC_PWM1->MR3 = -cycle * MOTOR_SCALE;
            }
            break;
        case MOTOR_FRONT_FRONT:
            if(cycle > 0)
            {
                set_gpio_pin(M3_DIR1, GPIO_OFF);
                set_gpio_pin(M3_DIR2, GPIO_ON);
                LPC_PWM1->MR6 = cycle * MOTOR_SCALE;
            }
            else
            {
                set_gpio_pin(M3_DIR1, GPIO_ON);
                set_gpio_pin(M3_DIR2, GPIO_OFF);
                LPC_PWM1->MR6 = -cycle * MOTOR_SCALE;
            }
            break;
        case MOTOR_FRONT_SIDE:
            if(cycle > 0)
            {
                set_gpio_pin(M4_DIR1, GPIO_OFF);
                set_gpio_pin(M4_DIR2, GPIO_ON);
                LPC_PWM1->MR5 = cycle * MOTOR_SCALE;
            }
            else
            {
                set_gpio_pin(M4_DIR1, GPIO_ON);
                set_gpio_pin(M4_DIR2, GPIO_OFF);
                LPC_PWM1->MR5 = -cycle * MOTOR_SCALE;
            }
            break;
    }

    LPC_PWM1->LER = LER3_EN | LER4_EN | LER5_EN | LER6_EN;
}

void set_motor_position(uint32_t channel, int32_t position, int8_t direction_speed)
{
    //position -= (position > 100) ? 100: 0;
    int32_t enc;
    motor[channel].desired_position = position;
    motor[channel].state = MOTOR_MOVING;
    while (position < 0) {
        position += ENCODER_RANGE;
    }
    position = position % ENCODER_RANGE;
    int32_t desired_position = position;
    enc = g_enc[channel];
    if (ABS(enc - desired_position) < MOTOR_CONTROL_TOLERANCE) {
        /* Turn the PID controller on */
        motor[channel].pid_integ = 0;
        motor[channel].pid_flag = 1;
    } else {
        set_motor_speed(channel, direction_speed);
    }
}

/* This function is basically the same as the above function, except it
 * figures out the directionality/sign of the speed automatically so produce
 * the shortest motion. This is particularly good for the body joints so that we
 * don't accidentally command the body joint to turn past its limits due to a
 * directionality error. A negative value for 'speed' will force the joint to
 * go "the long way" around.
 */
void set_motor_position_abs(uint32_t channel, int32_t position, int8_t speed)
{
	int8_t direction;
	int32_t enc;
    enc = g_enc[channel];
	/* Get both position and enc to a range of -180 <-> 180 */
	if(abs_angle_diff(enc, position) > 0) {
		direction = -1;
	} else {
		direction = 1;
	}
	set_motor_position(channel, position, speed*direction);
}

void MotorStart()
{
    /* All single edge, all enable */
    LPC_PWM1->PCR = PWMENA3 | PWMENA4 | PWMENA5 | PWMENA6;
    LPC_PWM1->TCR = TCR_CNT_EN | TCR_PWM_EN; /* counter enable, PWM enable */
}

void MotorStop()
{
    LPC_PWM1->PCR = 0;
    LPC_PWM1->TCR = 0x00; /* Stop all PWMs */
}

void wait_motor(uint32_t channel)
{
	while(motor[channel].state == MOTOR_MOVING);
}

void MotorHandler(void)
{
  int32_t enc[4];
	static int32_t last_enc[4];
    int i;
    double gain;
    int32_t err;
    int32_t pid_out;

    enc[MOTOR_BACK_FRONT] = EncoderRead(ENC_BACK_FRONT);
    enc[MOTOR_BACK_SIDE] = EncoderRead(ENC_BACK_SIDE);
    enc[MOTOR_FRONT_FRONT] = EncoderRead(ENC_FRONT_FRONT);
    enc[MOTOR_FRONT_SIDE] = EncoderRead(ENC_FRONT_SIDE);
    /* Fix the values of the fully rotation endplates, which are 1 to 4 */
    for(i = 2; i < 4; i++) {
        if(enc[i] < 300 && last_enc[i] > 2300) {
            motor[i].rotation++;
        }
        else if(enc[i] > 2300 && last_enc[i] < 300) {
            motor[i].rotation--;
        }
        last_enc[i] = enc[i];
        enc[i] = enc[i]/4 + motor[i].rotation*900;
        while(enc[i] < 0) {
            enc[i] += 3600;
        }
    }
    for(i = 0; i < 4; i++) {
        g_enc[i] = enc[i];
    }

    /* Before we do any comparisons, we need to make sure enc_xx is
     * within 180 degrees of desired_position. This is because a
     * "1" degree reading is actually close to "359" degrees, and
     * we must account for that either by changing "1"->"361" or "359"
     * to "-1". This will depend on how "desired_position" is specified
     */
    for(i = 0; i < 4; i++) {
    	while((enc[i] - motor[i].desired_position) > (ENCODER_RANGE/2)) {
    		/* enc is too high */
    		enc[i] -= ENCODER_RANGE;
    	} 
        while((enc[i] - motor[i].desired_position) < -(ENCODER_RANGE/2)) {
    		/* enc is too low */
    		enc[i] += ENCODER_RANGE;
    	}
    }
#ifdef VERBOSE
    count++;
    if(!(count%50)) {
    	printf("ENC: %4d %4d %4d %4d\n", enc[0], enc[1], enc[2], enc[3]);
    	printf("DES: %4d %4d %4d %4d\n", motor[0].desired_position,
									     motor[1].desired_position,
    									 motor[2].desired_position,
    									 motor[3].desired_position);
    }
#endif
    /* Check the position of each motor to see if it has reached its goal */
    for(i = 0; i < 4; i++) {
        if(motor[i].state == MOTOR_IDLE) {
            continue;
        }
        /* If both the "direction" and "speed" are set on the motor, do not
         * stop it. */
        if(motor[i].direction != MOTOR_DIR_AUTO && motor[i].speed != 0) {
            continue;
        }

        /* If the motor has reached its goal, stop it */
    	if (ABS(enc[i] - motor[i].desired_position) < MOTOR_GOAL_TOLERANCE)
    	{
            /* Stop the motor */
            set_motor_speed(i, 0);
            motor[i].state = MOTOR_IDLE;
            motor[i].pid_flag = 0;
            continue;
    	}

        /* Motor PID control */
        if(motor[i].pid_flag) {
            err = abs_angle_diff(motor[i].desired_position, enc[i]);
            gain = (double)motor[i].speed / (double)MOTOR_CONTROL_TOLERANCE;
            motor[i].pid_integ += motor[i].pid_integ_gain * err * g_motorHandlerTimestep;
            pid_out = ((double)err)*gain + motor[i].pid_integ;
            if(pid_out > motor[i].speed) {
                pid_out = motor[i].speed;
            }
            if(pid_out < -1*(int32_t)motor[i].speed) {
                pid_out = -1*(int32_t)motor[i].speed;
            }
            set_motor_speed(i, pid_out);
            continue;
        }

        /* If the motor gets "kind of" close, enable the PID controler for the
         * last few degrees */
    	if (ABS(enc[i] - motor[i].desired_position) < MOTOR_CONTROL_TOLERANCE)
    	{
            motor[i].pid_integ = 0;
            motor[i].pid_flag = 1;
    	}
    }
    /* Make sure the body joints do not go past their limits */
    if(enc[MOTOR_FRONT_SIDE] >= 90*ENCODER_MULTIPLIER 
        && 
        enc[MOTOR_FRONT_SIDE] <= 270*ENCODER_MULTIPLIER) 
    {
    	set_motor_speed(MOTOR_FRONT_SIDE, 0);
    	motor[MOTOR_FRONT_SIDE].state = MOTOR_IDLE;
    }
    if(enc[MOTOR_BACK_SIDE] >= 90*ENCODER_MULTIPLIER 
        && 
        enc[MOTOR_BACK_SIDE] <= 270*ENCODER_MULTIPLIER) 
    {
        set_motor_speed(MOTOR_BACK_SIDE, 0);
        motor[MOTOR_BACK_SIDE].state = MOTOR_IDLE;
    }
}

int32_t abs_angle_diff(int32_t a, int32_t b)
{
    /* Get the two numbers to within 180 of each other */
    while (((a - b) >= 180*ENCODER_MULTIPLIER) || ((b - a) >= (180*ENCODER_MULTIPLIER))) {
        if(a > b) {
            a -= 360*ENCODER_MULTIPLIER;
        } else {
            a += 360*ENCODER_MULTIPLIER;
        }
    }
    return a-b;
}


void motor_set_direction(uint32_t motor_index, uint32_t dir)
{
	// Set the "direction" register
	motor[motor_index].direction = dir;

	// If the new direction is not AUTO, start turning the motor in that
	// direction at the motor speed.
	if(motor[motor_index].direction == MOTOR_DIR_FORWARD) {
		set_motor_speed(motor_index, motor[motor_index].speed);
	} else if (motor[motor_index].direction == MOTOR_DIR_BACKWARD) {
		set_motor_speed(motor_index, -1*motor[motor_index].speed);
	} else {
		set_motor_position_abs(motor_index, motor[motor_index].desired_position, motor[motor_index].speed);
	}
}

void motor_set_speed(uint32_t motor_index, uint32_t speed)
{
	// Set the "speed" register
	motor[motor_index].speed = speed;

	// If setting the motor speed to zero, stop the motor immediately
	if(motor[motor_index].speed == 0) {
		set_motor_speed(motor_index, 0);
	}

	// Check the direction register. If set to AUTO, do not start moving the motor yet.
	if(motor[motor_index].direction == MOTOR_DIR_FORWARD) {
		set_motor_speed(motor_index, speed);
	} else if (motor[motor_index].direction == MOTOR_DIR_BACKWARD) {
		set_motor_speed(motor_index, -speed);
	}
}
