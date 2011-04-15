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

int main(void)
{
    SystemInit();
	GpioInit();

	uint32_t buf[10];
	uint32_t ret = 0;

	//AdcInit();

	//TimerInit(0, 100ul * _millisecond);
    //CallbackRegister(TestFunc, 500ul * _millisecond);
	//CallbackEnable(TestFunc);
	//TimerEnable(0);

	//MotorInit();
	//MotorStart();

	I2cInit(SENSOR_BUS);

	while (1)
	{
		buf[0] = 0;
		I2cSend(SENSOR_BUS,0x90,buf,1);
		ret = I2cRecieve(SENSOR_BUS,0x90,1);
		snprintf(buffer,30,"%d\n",ret);
		consoleprint(buffer);
	}
	return 0;
}

void TestFunc(void)
{
	set_gpio_pin(LED2, GPIO_TOGGLE);
    snprintf(buffer,30,"%d\n",AdcRead(ADC_BAT));
    consoleprint(buffer);
}
