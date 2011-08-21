
#include "global.h"
#include "motor.h"
#include "i2c_module.h"
#include "i2c_sensor.h"
#include "i2c.h"

#include "consoleprint.h"

uint32_t module_state = SLAVE_IDLE;
uint32_t module_reg = 0x00;

uint32_t module_select = 0x00;
uint32_t module_addr = 0x01;

uint32_t module_temp_polling_rate = 0;
uint32_t module_temp_supply_voltate = 0;
uint32_t module_temp_supply_current = 0;
uint32_t module_temp_peripheral_current = 0;

uint32_t module_temp_motor_home_position[4];
uint32_t module_temp_motor_position[4];

void module_init()
{
	// todo: load module_addr from nvram

    // enable I2C peripheral block
    LPC_SC->PCONP |= _BIT(PCI2C);

	LPC_SC->PCONP |= 1<<PCI2C0;
	LPC_PINCON->PINSEL1 &= ~(0x3<<22);
	LPC_PINCON->PINSEL1 |=  (0x1<<22);
	LPC_PINCON->PINSEL1 &= ~(0x3<<24);
	LPC_PINCON->PINSEL1 |=  (0x1<<24);
	//set_gpio_select(I2C_MOD_CLK, 1);
	//set_gpio_select(I2C_MOD_DATA, 1);
	//set_gpio_dir(I2C_MOD_CLK, GPIO_OUTPUT);
	//set_gpio_dir(I2C_MOD_DATA, GPIO_OUTPUT);
	//set_gpio_pull(I2C_MOD_CLK, 1);
	//set_gpio_pull(I2C_MOD_DATA, 1);
	//set_gpio_od(I2C_MOD_CLK, 1);
	//set_gpio_od(I2C_MOD_DATA, 1);
	/* Set pins for normal 100MHz operation */
	//LPC_PINCON->I2CPADCFG &= ~(0x0F);
	// Clear flags
	//LPC_I2C0->I2CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;
	// Set up the clock
	LPC_SC->PCLKSEL0 |= 0x3 << 14; /* Sets the peripheral clock to 100/8 = 12.5 MHz */
	// Reset registers
	//LPC_I2C0->I2SCLL = 100; //I2SCLL_SCLL;
	//LPC_I2C0->I2SCLH = 100; //I2SCLH_SCLH;
	LPC_I2C0->I2SCLL = 65; //I2SCLL_SCLL;
	LPC_I2C0->I2SCLH = 65; //I2SCLH_SCLH;
	LPC_I2C0->I2ADR0 = module_addr<<1;
								// because first bit is reserved for
								// "general call enable". Sec 19.8.7
	LPC_I2C0->I2MASK0 = 0x7F<<1; // Address mask

	// Install interrupt handler
	NVIC_EnableIRQ(I2C0_IRQn);

	LPC_I2C0->I2CONSET = I2CONSET_I2EN | I2CONSET_AA;
}

void I2C0_IRQHandler(void) // MODULE
{
    switch(LPC_I2C0->I2STAT)
    {
		case 0x0: /* Error */
			LPC_I2C0->I2CONSET = 0x14;
			break;

		case SR_ADDRESSED:
			reply_ack();
			break;

		case SR_GEN_CALL:
		case SR_DATA_RECV_ACK:
			switch (module_state) {
				case SLAVE_IDLE:
					module_reg = LPC_I2C0->I2DAT;
					module_state = SLAVE_RECV;
					reply_ack();
					break;
				case SLAVE_RECV:
					slave_write_register(module_reg, LPC_I2C0->I2DAT);
					module_state = SLAVE_IDLE;
					reply_nack();
				default:
					reply_nack();
					break;
			}
			break;

		case SR_GEN_CALL_DATA:
			break;
		case SR_STOP:
			reply_ack();
			break;

		case ST_ADDRESSED:
			/* Make sure we are in the "SLAVE_RECV" state, which means a valid
			 * address has been loaded into the register. */
			if (module_state != SLAVE_RECV) {
				reply_nack();
				module_state = SLAVE_IDLE;
				break;
			} else {
				LPC_I2C0->I2DAT = slave_read_register(module_reg);
				module_state = SLAVE_IDLE;
				reply_ack();
				break;
			}
			break;
		case ST_TRANSMIT_ACK:
		case ST_TRANSMIT_NACK:
			reply_ack();
			module_state = SLAVE_IDLE;
			break;
		case MT_ARB_LOST:
		default:
			break;
	}
	LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
}

#define I2C_MOTOR_SPEED 20
void slave_write_register(uint8_t reg, uint8_t dat)
{
	// check if it's the special case device select register first
	if (reg == MODULE_REG_DEVICE_SELECT)
	{
		module_select = dat;
		return;
	}

	// if a peripheral is selected, make the changes there
	if (module_select != 0)
	{
		sensor_set_value(module_select, reg, dat);
		return;
	}

	if (reg == MODULE_REG_DEVICE_ADDRESS)
	{
		module_addr = dat;
		// todo: save to nvram
	}
	else if (reg == MODULE_REG_PERIPHERAL_ENABLE)
	{
		// todo: handle enabling/disabling peripherals
	}
	else if (reg == MODULE_REG_PERIPHERAL_UPDATE_MODE)
	{
		sensor_set_polling_mode(dat);
	}
	else if (reg == MODULE_REG_PERIPHERAL_POLLING_RATE)
	{
		module_temp_polling_rate = (module_temp_polling_rate & 0x00ff) | (dat << 8);
	}
	else if (reg == MODULE_REG_PERIPHERAL_POLLING_RATE+1)
	{
		module_temp_polling_rate = (module_temp_polling_rate & 0xff00) | (dat);
		sensor_set_polling_rate(dat);
	}
	else if (reg >= MODULE_REG_MOTOR_BLOCK_START && reg <= MODULE_REG_MOTOR_BLOCK_END)
	{
		uint8_t motor_index = (reg-MODULE_REG_MOTOR_BLOCK_START) >> 4;
		uint8_t motor_reg = (reg-MODULE_REG_MOTOR_BLOCK_START) & 0x0F;

		if (motor_reg == MODULE_MOTOR_REG_POSITION)
		{
			module_temp_motor_position[motor_index] &= 0x00FF;
			module_temp_motor_position[motor_index] |= dat<<8;

			// If the highest bit is set, it must be a negative number.
			if (dat & 0x80) {
				motor[motor_index].desired_position |= 0xFFFF0000;
			}
		}
		else if (motor_reg == MODULE_MOTOR_REG_POSITION+1)
		{
			module_temp_motor_position[motor_index] &= 0xFFFFFF00;
			module_temp_motor_position[motor_index] |= (uint16_t) (0x00FF & dat);

			motor[motor_index].desired_position = module_temp_motor_position[motor_index];

			// If the motor direction is AUTO, start moving the motor towards the goal
			if(motor[motor_index].direction == MOTOR_DIR_AUTO) {
				set_motor_position_abs(motor_index, motor[motor_index].desired_position, motor[motor_index].speed);
			}
		}
		else if (motor_reg == MODULE_MOTOR_REG_DIRECTION)
		{
			motor_set_direction(motor_index, dat);
		}
		else if (motor_reg == MODULE_MOTOR_REG_SPEED)
		{
			motor_set_speed(motor_index, dat);
		}
	}
}

uint32_t slave_read_register(uint8_t reg)
{
	// check if it's the special case device select register first
	if (reg == MODULE_REG_DEVICE_SELECT)
	{
		return module_select;
	}

	// if a peripheral is selected, make the changes there
	if (module_select != 0)
	{
		return sensor_get_value(module_select, reg);
	}

	if (reg == MODULE_REG_DEVICE_ADDRESS)
	{
		return module_addr;
	}
	else if (reg == MODULE_REG_PERIPHERAL_ENABLE)
	{
		// todo: handle enabling/disabling peripherals
	}
	else if (reg == MODULE_REG_PERIPHERAL_UPDATE_MODE)
	{
		return sensor_get_polling_mode();
	}
	else if (reg == MODULE_REG_PERIPHERAL_POLLING_RATE)
	{
		module_temp_polling_rate = sensor_get_polling_rate();
		return module_temp_polling_rate >> 8;
	}
	else if (reg == MODULE_REG_PERIPHERAL_POLLING_RATE+1)
	{
		return module_temp_polling_rate & 0xff;
	}
	else if (reg == MODULE_REG_SUPPLY_VOLTAGE)
	{
		// todo: calculate supply voltage
		module_temp_supply_voltate = 0x4243;
		return module_temp_supply_voltate >> 8;
	}
	else if (reg == MODULE_REG_SUPPLY_VOLTAGE + 1)
	{
		return module_temp_supply_voltate & 0xFF;
	}
	else if (reg == MODULE_REG_SUPPLY_CURRENT)
	{
		// todo: calculate supply current
		module_temp_supply_current = 0x4445;
	}
	else if (reg == MODULE_REG_SUPPLY_CURRENT + 1)
	{
		return module_temp_supply_voltate & 0xFF;
	}
	else if (reg == MODULE_REG_PERIPHERAL_BUS_CURRENT)
	{
		// todo: calculate peripheral bus current
		module_temp_peripheral_current = 0x4647;
	}
	else if (reg == MODULE_REG_PERIPHERAL_BUS_CURRENT + 1)
	{
		return module_temp_supply_voltate & 0xFF;
	}
	else if (reg >= MODULE_REG_MOTOR_BLOCK_START && reg <= MODULE_REG_MOTOR_BLOCK_END)
	{
		uint8_t motor_index = (reg-MODULE_REG_MOTOR_BLOCK_START) >> 4;
		uint8_t motor_reg = (reg-MODULE_REG_MOTOR_BLOCK_START) & 0x0F;

		if (motor_reg == MODULE_MOTOR_REG_POSITION)
		{
			module_temp_motor_position[motor_index] = g_enc[motor_index];
			return 0x00FF & (module_temp_motor_position[motor_index]>>8);
		}
		else if (motor_reg == MODULE_MOTOR_REG_POSITION)
		{
			return 0x00FF & module_temp_motor_position[motor_index];
		}
		else if (motor_reg == MODULE_MOTOR_REG_DIRECTION)
		{
			return 0x00FF & (motor[motor_index].direction);
		}
		else if (motor_reg == MODULE_MOTOR_REG_SPEED)
		{
			return 0x00FF & (motor[motor_index].speed);
		}
	}

	return 0;
}

void slave_reset()
{
	LPC_I2C0->I2CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;
	LPC_I2C0->I2CONSET = I2CONSET_I2EN | I2CONSET_AA;
}

void reply_ack()
{
	LPC_I2C0->I2CONSET = I2CONSET_AA;
}

void reply_nack()
{
	LPC_I2C0->I2CONCLR = I2CONSET_AA | I2CONSET_SI;
}
