#ifndef HARDWARE_DEF_H_
#define HARDWARE_DEF_H_

#include "global.h"

#define _BIT(x)  (1<<(x))

#define BETWEEN(X,Y,Z) ((Y) > (X) && (Y) < (Z))

void set_gpio_select(int id, int function);
void set_gpio_pull(int id, int up);
void set_gpio_od(int id, int od);
void set_gpio_dir(int id, int state);
void set_gpio_pin(int id, int state);
void clr_gpio_pin(int id);
int get_gpio_pin(int id);
void GpioInit(void);

enum
{
    GPIO_INPUT = 0, GPIO_OUTPUT = 1
};
enum
{
    GPIO_ON, GPIO_OFF, GPIO_TOGGLE
};

// I2C
#define I2C_SEN_DATA      0
#define I2C_SEN_CLK       1
#define I2C_EEPROM_DATA  10
#define I2C_EEPROM_CLK   11
#define I2C_MOD_DATA     27
#define I2C_MOD_CLK      28

// Motors
enum
{
    MOTOR_FRONT_SIDE = 0,
    MOTOR_BACK_SIDE,
    MOTOR_FRONT_FRONT,
    MOTOR_BACK_FRONT
};

#define M12_STBY        208
#define M1_PWM          203
#define M1_DIR1         209
#define M1_DIR2          16
#define M2_PWM          202
#define M2_DIR1         207
#define M2_DIR2         206

#define M23_STBY         19
#define M3_PWM          205
#define M3_DIR1          20
#define M3_DIR2          21
#define M4_PWM          204
#define M4_DIR1          18
#define M4_DIR2          17

// Current/Voltage Sensors
enum
{
    ADC_BAT = 0, ADC_I_VCC = 1, ADC_I_BUS = 2
};
#define VSENSE_BAT       23
#define ISENSE_VCC       24
#define ISENSE_BUS       25

// Encoders
enum
{
    ENC_BS = 3, ENC_BF = 2, ENC_FS = 0, ENC_FF = 1
};

enum
{
    ADC_IR_BACK = 4, ADC_IR_FRONT = 5
};
#define VSENSE_IR_BACK  130
#ifdef REWORKED_ENCODER_BOARD
    #define ENC_FRONT_SIDE   129
#else
    #define ENC_FRONT_SIDE   210
#endif
#define ENC_FRONT_FRONT  211

#define VSENSE_IR_FRONT 131
#define ENC_BACK_FRONT 212
#define ENC_BACK_SIDE  213

// Sensor bus enables
#define SEN_1_EN        123
#define SEN_2_EN        122
#define SEN_3_EN        121
#define SEN_4_EN        120
#define SEN_5_EN        119
#define SEN_6_EN        118

// LEDs
#define RED_LED         100
#define GREEN_LED       101

#ifdef LPC_X_DEBUG
#define LED2             22
#endif

// Uart
#define CONSOLE_TX      200
#define CONSOLE_RX      201

#endif /* HARDWARE_DEF_H_ */
