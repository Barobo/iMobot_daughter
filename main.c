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

	InitTimer(0, 1ul * _1_MS);
    RegisterCallback(ServiceEncoder, 1000ul);
	EnableCallback(ServiceEncoder);
	EnableTimer(0);

	MotorInit();
	MotorStart();

	while(1);
	return 0 ;
}

void ServiceEncoder(void)
{
    snprintf(buffer,30,"%d\n",get_encoder_duty_percent(1));
    consoleprint(buffer);
}
