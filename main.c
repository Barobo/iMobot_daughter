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

void TestFunc(void);

char buffer[30];
extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint32_t I2CCmd, I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

int main(void)
{
	uint32_t i = 0;
    SystemInit();
	GpioInit();
	//AdcInit();

	//TimerInit(0, 100ul * _millisecond);
    //CallbackRegister(TestFunc, 500ul * _millisecond);
	//CallbackEnable(TestFunc);
	//TimerEnable(0);

	//MotorInit();
	//MotorStart();
	if (I2CInit((uint32_t)I2CMASTER) == FALSE){	while (1); }
	/* Configure temp register before reading */

	while (1)
	{
		  for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */
		  {
			I2CMasterBuffer[i] = 0;
		  }
		  I2CWriteLength = 1;
		  I2CReadLength = 0;
		  I2CMasterBuffer[0] = 0x90;
		  I2CMasterBuffer[1] = 0x00;
		  I2CMasterBuffer[2] = 0x00;	/* configuration value, no change from
										default */
		  I2CCmd = 0x00;
		  I2CEngine();

		  /* Get temp reading */
		  for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */
		  {
			I2CMasterBuffer[i] = 0;
		  }
		  I2CWriteLength = 1;
		  I2CReadLength = 1;
		  I2CMasterBuffer[0] = 0x91;
		  I2CMasterBuffer[1] = 0x00;
		  I2CMasterBuffer[2] = 0x00;
		  I2CCmd = 0x01;
		  I2CEngine();

		snprintf(buffer,30,"%d %d %d %d\n",I2CMasterBuffer[0],I2CMasterBuffer[1],I2CMasterBuffer[2],I2CMasterBuffer[3]);
		consoleprint(buffer);
	}
	while(1);
	return 0;
}

void TestFunc(void)
{
	set_gpio_pin(LED2, GPIO_TOGGLE);
    snprintf(buffer,30,"%d\n",AdcRead(ADC_BAT));
    consoleprint(buffer);
}
