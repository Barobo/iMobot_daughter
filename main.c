#include "main.h"
#include "global.h"
#include "hardware_def.h"
#include "timer.h"
#include "callback.h"
#include "consoleprint.h"
#include "motor.h"
#include "adc.h"

void TestFunc(void);

char buffer[30];

int main(void)
{
    SystemInit();
	GpioInit();
	AdcInit();

	TimerInit(0, 100ul * _millisecond);
    CallbackRegister(TestFunc, 500ul * _millisecond);
	CallbackEnable(TestFunc);
	TimerEnable(0);

	MotorInit();
	MotorStart();

	while(1);
	return 0;
}

void TestFunc(void)
{
	set_gpio_pin(LED2, GPIO_TOGGLE);
    snprintf(buffer,30,"%d\n",AdcRead(ADC_BAT));
    consoleprint(buffer);
}
