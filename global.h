#ifndef GLOBAL_H
#define GLOBAL_H

#define MAIN_CLOCK      100000000 // 100 MHz
#define TRUE 1
#define FALSE 0

// compile flags
#define LPC_X_DEBUG //LPCxpresso debug board

#define REWORKED_ENCODER_BOARD
//#define VERBOSE

// these must follow compiler flags
#include "LPC17xx.h"
#include "system_LPC17xx.h"
#include "stdint.h"
#include "hardware_def.h"

#define ABS(a)     (((a) < 0) ? -(a) : (a))

/* Center positions for each joint */
#define BS_CENTER 49
#define BF_CENTER 277
#define FS_CENTER 260
#define FF_CENTER 29

#endif //GLOBAL_H
