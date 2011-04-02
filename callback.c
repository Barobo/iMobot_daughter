#include "callback.h"
#include "timer.h"
#include "global.h"

uint8_t event_count = 0;
volatile uint32_t callback_mult = 1;
ScheduledEvent event_list[MAX_CALLBACK_CNT];

void RegisterCallback(SchedulerCallback callbackFunction, uint32_t run_time)
{
    if (event_count < sizeof(event_list) / sizeof(SchedulerCallback))
    {
        event_list[event_count].enabled       = FALSE;
        event_list[event_count].func          = callbackFunction;
        event_list[event_count].run_time      = run_time;
        event_list[event_count].next_run_time = now + run_time;
        event_count++;
    }
}

void RunCallbacks(uint32_t current_time)
{
    uint8_t i = 0;
    for (i = 0;i < event_count;i++)
    {
        if (event_list[i].enabled == TRUE && now == event_list[i].next_run_time)
        {
            event_list[i].next_run_time = current_time + event_list[i].run_time;
            event_list[i].func();
        }
    }
}

void EnableCallback(SchedulerCallback func)
{
    uint8_t i = 0;
    for (i = 0;i < event_count;i++)
    {
        if (func == event_list[i].func)
        {
            event_list[i].enabled = TRUE;
            event_list[i].next_run_time = now + event_list[i].run_time;
            break;
        }
    }
}

void DisableCallback(SchedulerCallback func)
{
    uint8_t i = 0;
    for (i = 0;i < event_count;i++)
    {
        if (func == event_list[i].func)
        {
            event_list[i].enabled = FALSE;
            break;
        }
    }
}
