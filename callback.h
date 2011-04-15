#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "global.h"

typedef void (*SchedulerCallback)(void);

#define MAX_CALLBACK_CNT 5

enum
{
    DISABLED = 0, ENABLED = 1
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
int32_t CallbackRegister(SchedulerCallback callbackFunction, uint32_t run_time);
void CallbackService(uint32_t current_time);
int32_t CallbackEnable(SchedulerCallback func);
int32_t CallbackDisable(SchedulerCallback func);
void set_callback_divisor(uint32_t interval);

extern volatile uint32_t callback_divisor;

// Base timing for callbacks (minimum 1uS)
#define _microsecond	(1)
#define _millisecond	(1000)
#define _second    		(1000000)
#define _minute    		(60000000)

#endif
