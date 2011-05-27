#include "global.h"
#include "motor.h"
#include "encoder.h"
#include "consoleprint.h"

motor_t motor[4];

#define MOTOR_GOAL_TOLERANCE 10

uint32_t MotorInit()
{
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
    return 1;
}

#define MOTOR_SCALE 100
void set_motor_speed(uint32_t channel, int cycle)
{

    switch(channel)
    {
        case 0:
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
        case 1:
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
        case 2:
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
        case 3:
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
	while (position < 0) {
		position += 360;
	}
	position = position % 360;
    motor[channel].desired_position = position;
    motor[channel].state = MOTOR_MOVING;
    switch(channel)
    {
        case MOTOR_FRONT_FRONT:
        	enc = EncoderRead(ENC_FRONT_FRONT);
        	if (ABS(enc - motor[MOTOR_FRONT_FRONT].desired_position) < MOTOR_GOAL_TOLERANCE) {
        		motor[channel].state = MOTOR_IDLE;
        		break;
        	}
            set_motor_speed(MOTOR_FRONT_FRONT, direction_speed);
            break;
        case MOTOR_FRONT_SIDE:
        	enc = EncoderRead(ENC_FRONT_SIDE);
        	if (ABS(enc - motor[MOTOR_FRONT_SIDE].desired_position) < MOTOR_GOAL_TOLERANCE) {
        		motor[channel].state = MOTOR_IDLE;
        		break;
        	}
            set_motor_speed(MOTOR_FRONT_SIDE, direction_speed);
            break;
        case MOTOR_BACK_FRONT:
        	enc = EncoderRead(ENC_BACK_FRONT);
        	if (ABS(enc - motor[MOTOR_BACK_FRONT].desired_position) < MOTOR_GOAL_TOLERANCE) {
        		motor[channel].state = MOTOR_IDLE;
        	    break;
        	}
            set_motor_speed(MOTOR_BACK_FRONT, direction_speed);
            break;
        case MOTOR_BACK_SIDE:
        	enc = EncoderRead(ENC_BACK_SIDE);
        	if (ABS(enc - motor[MOTOR_BACK_SIDE].desired_position) < MOTOR_GOAL_TOLERANCE) {
        		motor[channel].state = MOTOR_IDLE;
        	    break;
        	}
            set_motor_speed(MOTOR_BACK_SIDE, direction_speed);
            break;
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
	switch(channel) {
	case MOTOR_FRONT_FRONT:
		enc = EncoderRead(ENC_FRONT_FRONT);
		break;
	case MOTOR_FRONT_SIDE:
		enc = EncoderRead(ENC_FRONT_SIDE);
		break;
	case MOTOR_BACK_SIDE:
		enc = EncoderRead(ENC_BACK_SIDE);
		break;
	case MOTOR_BACK_FRONT:
		enc = EncoderRead(ENC_BACK_FRONT);
		break;
	}
	/* Get both position and enc to a range of -180 <-> 180 */
	while(position > 180) position -= 360;
	while(position <= -180) position += 360;
	while(enc > 180) enc -= 360;
	if(enc > position) {
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
    static int count;

    enc[MOTOR_BACK_FRONT] = EncoderRead(ENC_BACK_FRONT);
    enc[MOTOR_BACK_SIDE] = EncoderRead(ENC_BACK_SIDE);
    enc[MOTOR_FRONT_FRONT] = EncoderRead(ENC_FRONT_FRONT);
    enc[MOTOR_FRONT_SIDE] = EncoderRead(ENC_FRONT_SIDE);


    /* Before we do any comparisons, we need to make sure enc_xx is
     * within 180 degrees of desired_position. This is because a
     * "1" degree reading is actually close to "359" degrees, and
     * we must account for that either by changing "1"->"361" or "359"
     * to "-1". This will depend on how "desired_position" is specified
     */
    for(i = 0; i < 4; i++) {
    	if((enc[i] - motor[i].desired_position) > 180) {
    		/* enc is too high */
    		enc[i] -= 360;
    	} else if ((enc[i] - motor[i].desired_position) < -180) {
    		/* enc is too low */
    		enc[i] += 360;
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
    	if (ABS(enc[i] - motor[i].desired_position) < MOTOR_GOAL_TOLERANCE)
    	{
    		set_motor_speed(i,0);
    		motor[i].state = MOTOR_IDLE;
    	}
    	/* Try and detect if we have overshot our mark */
#if 0
    	if (
    			(last_enc[i] > motor[i].desired_position && enc < motor[i].desired_position) ||
    			(last_enc[i] < motor[i].desired_position && enc > motor[i].desired_position) )
    	{
    		printf(".");
    		set_motor_speed(i, 0);
    		motor[i].state = MOTOR_IDLE;
    	}
    	last_enc[i] = enc[i];
#endif
    }
#if 0
    /* Make sure the body joints do not go past their limits */
    if(enc[MOTOR_FRONT_SIDE] >= 90 && enc[MOTOR_FRONT_SIDE] <= 270) {
    	set_motor_speed(MOTOR_FRONT_SIDE, 0);
    	motor[MOTOR_FRONT_SIDE].state = MOTOR_IDLE;
    }
    if(enc[MOTOR_BACK_SIDE] >= 90 && enc[MOTOR_BACK_SIDE] <= 270) {
        set_motor_speed(MOTOR_BACK_SIDE, 0);
        motor[MOTOR_BACK_SIDE].state = MOTOR_IDLE;
    }
#endif
}
