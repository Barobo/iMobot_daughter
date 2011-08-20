/*
 * i2c_sensor.h
 *
 *  Created on: Aug 19, 2011
 *      Author: kevin
 */

#ifndef I2C_SENSOR_H_
#define I2C_SENSOR_H_

#include "i2c.h"

#define NUM_SENSORS 6

enum
{
    SENSOR_ADDR_ADDRESS = 0x01,
    SENSOR_ADDR_TYPE = 0x02,
    SENSOR_ADDR_NUM_READ_ENDPOINTS = 0x03,
    SENSOR_ADDR_START_READ_ENDPOINTS = 0x04,
    SENSOR_ADDR_NUM_WRITE_ENDPOINTS = 0x05,
    SENSOR_ADDR_START_WRITE_ENDPOINTS = 0x06,
};
extern void sensor_init();
extern uint32_t sensor_start();
extern uint32_t sensor_stop();
extern uint32_t sensor_engine();
extern void sensor_write(uint32_t address, uint32_t reg, uint32_t val);
extern uint32_t sensor_read(uint32_t address, uint32_t reg);
extern void sensor_poll_loop();

extern void I2C1_IRQHandler(void);

#endif /* I2C_SENSOR_H_ */
