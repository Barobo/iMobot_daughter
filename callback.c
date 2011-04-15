#include "callback.h"
#include "timer.h"
#include "global.h"

uint8_t event_count = 0;
volatile uint32_t callback_divisor = 1;
ScheduledEvent event_list[MAX_CALLBACK_CNT];

int32_t CallbackRegister(SchedulerCallback callbackFunction, uint32_t run_time)
{
    // if the run time is less than the timer period, kick out
    if(run_time < callback_divisor)
    {
        return (-1);
    }
    if(event_count < sizeof(event_list) / sizeof(SchedulerCallback))
    {
        event_list[event_count].enabled = FALSE;
        event_list[event_count].func = callbackFunction;
        event_list[event_count].run_time = run_time / callback_divisor;
        event_list[event_count].next_run_time = now + run_time
                / callback_divisor;
        event_count++;
        return (0);
    }
    else
    {
        return (-1);
    }
}

void CallbackService(uint32_t current_time)
{
    uint8_t i = 0;
    for(i = 0;i < event_count;i++)
    {
        if(event_list[i].enabled == TRUE && now == event_list[i].next_run_time)
        {
            event_list[i].next_run_time = current_time + event_list[i].run_time;
            event_list[i].func();
        }
    }
}

int32_t CallbackEnable(SchedulerCallback func)
{
    uint8_t i = 0;
    for(i = 0;i < event_count;i++)
    {
        if(func == event_list[i].func)
        {
            event_list[i].enabled = TRUE;
            event_list[i].next_run_time = now + event_list[i].run_time;
            return (0);
        }
    }
    return (-1);
}

int32_t CallbackDisable(SchedulerCallback func)
{
    uint8_t i = 0;
    for(i = 0;i < event_count;i++)
    {
        if(func == event_list[i].func)
        {
            event_list[i].enabled = FALSE;
            return (0);
        }
    }
    return (-1);
}

void set_callback_divisor(uint32_t interval)
{
    callback_divisor = interval;
}
