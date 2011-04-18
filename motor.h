#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_MAX_SPEED 80

#define PWM_CYCLE		1200
#define PWM_OFFSET		200

#define MR0_INT			1 << 0
#define MR1_INT			1 << 1
#define MR2_INT			1 << 2
#define MR3_INT			1 << 3
#define MR4_INT			1 << 8
#define MR5_INT			1 << 9
#define MR6_INT			1 << 10

#define TCR_CNT_EN		0x00000001
#define TCR_RESET		0x00000002
#define TCR_PWM_EN		0x00000008

#define PWMMR0I			1 << 0
#define PWMMR0R			1 << 1
#define PWMMR0S			1 << 2
#define PWMMR1I			1 << 3
#define PWMMR1R			1 << 4
#define PWMMR1S			1 << 5
#define PWMMR2I			1 << 6
#define PWMMR2R			1 << 7
#define PWMMR2S			1 << 8
#define PWMMR3I			1 << 9
#define PWMMR3R			1 << 10
#define PWMMR3S			1 << 11
#define PWMMR4I			1 << 12
#define PWMMR4R			1 << 13
#define PWMMR4S			1 << 14
#define PWMMR5I			1 << 15
#define PWMMR5R			1 << 16
#define PWMMR5S			1 << 17
#define PWMMR6I			1 << 18
#define PWMMR6R			1 << 19
#define PWMMR6S			1 << 20

#define PWMSEL2			1 << 2
#define PWMSEL3			1 << 3
#define PWMSEL4			1 << 4
#define PWMSEL5			1 << 5
#define PWMSEL6			1 << 6
#define PWMENA1			1 << 9
#define PWMENA2			1 << 10
#define PWMENA3			1 << 11
#define PWMENA4			1 << 12
#define PWMENA5			1 << 13
#define PWMENA6			1 << 14

#define LER0_EN			1 << 0
#define LER1_EN			1 << 1
#define LER2_EN			1 << 2
#define LER3_EN			1 << 3
#define LER4_EN			1 << 4
#define LER5_EN			1 << 5
#define LER6_EN			1 << 6

typedef struct
{
    uint8_t state;
    uint8_t desired_position;
}motor_t;

extern motor_t motor[4];
// states
enum
{
    MOTOR_IDLE = 0,
    MOTOR_MOVING = 1,
};

extern uint32_t MotorInit();
extern void set_motor_speed(uint32_t channel, int cycle);
void set_motor_position(uint32_t channel, uint32_t position, int8_t direction_speed);
extern void MotorStart();
extern void MotorStop();
extern void MotorHandler(void);

#endif /* MOTOR_H_ */
