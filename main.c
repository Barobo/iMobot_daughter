#include "main.h"
#include "global.h"
#include "hardware_def.h"
#include "timer.h"
#include "callback.h"
#include "consoleprint.h"
#include "motor.h"

void ServiceEncoder(void);

char buffer[30];

int main(void)
{
    SystemInit();
	InitGpio();

	InitTimer(0, 100ul * _millisecond);
    RegisterCallback(ServiceEncoder, 500ul * _millisecond);
	EnableCallback(ServiceEncoder);
	EnableTimer(0);

	MotorInit();
	MotorStart();

	while(1);
	return 0;
}

void ServiceEncoder(void)
{
	set_gpio_pin(LED2, GPIO_TOGGLE);
    //snprintf(buffer,30,"%d\n",get_encoder_duty_percent(1));
    //consoleprint(buffer);
}
