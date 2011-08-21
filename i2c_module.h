/*
 * i2c_module.h
 *
 *  Created on: Aug 20, 2011
 *      Author: kevin
 */

#ifndef I2C_MODULE_H_
#define I2C_MODULE_H_

enum slave_state_e
{
	SLAVE_IDLE,
	SLAVE_RECV,
	SLAVE_SEND
};

enum module_registers
{
	MODULE_REG_DEVICE_SELECT = 0x00,
	MODULE_REG_DEVICE_ADDRESS = 0x01,
	MODULE_REG_PERIPHERAL_ENABLE = 0x10,
	MODULE_REG_PERIPHERAL_UPDATE_MODE = 0x11,
	MODULE_REG_PERIPHERAL_POLLING_RATE = 0x12,
	MODULE_REG_SUPPLY_VOLTAGE = 0x20,
	MODULE_REG_SUPPLY_CURRENT = 0x22,
	MODULE_REG_PERIPHERAL_BUS_CURRENT =0x24,
	MODULE_REG_MOTOR_BLOCK_START = 0x30,
	MODULE_REG_MOTOR_BLOCK_END = 0x69
};

enum module_motor_registers
{
	MODULE_MOTOR_REG_HOME_POSITION = 0x00,
	MODULE_MOTOR_REG_POSITION = 0x02,
	MODULE_MOTOR_REG_DIRECTION = 0x04,
	MODULE_MOTOR_REG_SPEED = 0x05,
};

extern void module_init();
extern void I2C0_IRQHandler(void);

void slave_write_register(uint8_t reg, uint8_t dat);
uint32_t slave_read_register(uint8_t reg);
void reply_ack();
void reply_nack();


#endif /* I2C_MODULE_H_ */
