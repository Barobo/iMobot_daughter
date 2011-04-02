#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "global.h"

typedef void (*SchedulerCallback)(void);

#define MAX_CALLBACK_CNT 5

enum
{
    DISABLED = 0,
    ENABLED = 1
};

typedef struct
{
    SchedulerCallback func;
    uint8_t enabled;
    uint32_t run_time;
    uint32_t next_run_time;
    uint8_t location;
} ScheduledEvent;

// Prototypes
void RegisterCallback(SchedulerCallback callbackFunction, uint32_t run_time);
void RunCallbacks(uint32_t current_time);
void EnableCallback(SchedulerCallback func);
void DisableCallback(SchedulerCallback func);

extern volatile uint32_t callback_mult;

// callbacks assume a base interrupt time of 1uS
#define _microsecond	(1 * callback_mult)
#define _millisecond	(10 * callback_mult)
#define _second    		(10000 * callback_mult)
#define _minute    		(600000 * callback_mult)

#endif
