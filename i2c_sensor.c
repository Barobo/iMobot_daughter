
#include "global.h"
#include "motor.h"
#include "i2c_sensor.h"
#include "i2c.h"
#include "timer.h"

#include "consoleprint.h"

volatile i2c_bus_t sensor_bus;
uint32_t sensor_memory[NUM_SENSORS+1][256];
uint32_t sensor_polling_mode = SENSOR_POLLING_MODE_POLL;
uint32_t sensor_polling_rate = 100;

void sensor_init()
{
    // enable I2C peripheral block
    LPC_SC->PCONP |= _BIT(PCI2C);

	set_gpio_select(I2C_SEN_CLK, 3);
	set_gpio_select(I2C_SEN_DATA, 3);
	// Clear flags
	LPC_I2C1->I2CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;
	// Reset registers
	LPC_I2C1->I2SCLL = I2SCLL_SCLL;
	LPC_I2C1->I2SCLH = I2SCLH_SCLH;
	// Install interrupt handler
	NVIC_EnableIRQ(I2C1_IRQn);

	LPC_I2C1->I2CONSET = I2CONSET_I2EN;

    // init variables
    sensor_bus.master_state = I2C_IDLE;
    sensor_bus.slave_state = I2C_IDLE;
    sensor_bus.count = 0;
    sensor_bus.read_index = 0;
    sensor_bus.write_index = 0;
}

uint32_t sensor_start()
{
    uint32_t timeout = 0;

    // send start
    LPC_I2C1->I2CONSET = I2CONSET_STA; // Set Start flag

    while (timeout++ < MAX_TIMEOUT)
    {
        if (sensor_bus.master_state == I2C_STARTED)
            return TRUE;
    }
    return FALSE;
}

uint32_t sensor_stop()
{
    LPC_I2C1->I2CONSET = I2CONSET_STO; /* Set Stop flag */
    LPC_I2C1->I2CONCLR = I2CONCLR_SIC; /* Clear SI flag */

    // Wait for stop
    while(LPC_I2C1->I2CONSET & I2CONSET_STO)
    {
        continue;
    }
    return TRUE;
}

void sensor_write(uint32_t address, uint32_t reg, uint32_t val)
{
	sensor_bus.write_length = 3; // address
	sensor_bus.read_length = 0;

	sensor_bus.master_buffer[0] = address<<1;
	sensor_bus.master_buffer[1] = reg;
	sensor_bus.master_buffer[2] = val;

	sensor_bus.command = 0;
    sensor_engine();
}

uint32_t sensor_read(uint32_t address, uint32_t reg)
{
	sensor_bus.write_length = 2; // address
	sensor_bus.read_length = 1;

	sensor_bus.master_buffer[0] = address<<1;
	sensor_bus.master_buffer[1] = reg;
	sensor_bus.master_buffer[2] = 0;

	sensor_bus.command = 0;
    sensor_engine();

    sensor_memory[address][reg] = sensor_bus.master_buffer[2];
    return sensor_bus.master_buffer[2];
}

uint32_t sensor_engine()
{
	sensor_bus.master_state = I2C_IDLE;
	sensor_bus.read_index = 0;
	sensor_bus.write_index = 0;

    if (sensor_start() != TRUE)
    {
        sensor_stop();
        return FALSE;
    }

    while (sensor_bus.master_state != DATA_NACK)
    {
        continue;
    }
    sensor_stop();

    return (TRUE);
}

void I2C1_IRQHandler(void) // SENSORS
{
    switch(LPC_I2C1->I2STAT)
    {
        case MT_START:
            LPC_I2C1->I2DAT = sensor_bus.master_buffer[0];
            LPC_I2C1->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
            sensor_bus.master_state = I2C_STARTED;
            break;
        case MT_REPEAT_START:
			LPC_I2C1->I2DAT = sensor_bus.master_buffer[0] | READ_BIT;
            LPC_I2C1->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
            sensor_bus.master_state = I2C_RESTARTED;
            break;
        case MT_DATA_ACK:
            if(sensor_bus.master_state == I2C_STARTED)
            {
                LPC_I2C1->I2DAT = sensor_bus.master_buffer[1 + sensor_bus.write_index];
                sensor_bus.write_index++;
                sensor_bus.master_state = DATA_ACK;
            }
            LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
            break;
        case MT_TRANSMIT_ACK:
        case MT_TRANSMIT_NACK:
            if(1 + sensor_bus.write_index < sensor_bus.write_length)
            {
                // this should be the last one
                LPC_I2C1->I2DAT = sensor_bus.master_buffer[1 + sensor_bus.write_index];
                sensor_bus.write_index++;
                if(sensor_bus.write_index < sensor_bus.write_length)
                {
                    sensor_bus.master_state = DATA_ACK;
                }
                else
                {
                    if(sensor_bus.read_length != 0)
                    {
                        LPC_I2C1->I2CONSET = I2CONSET_STA; // Set Repeated-start flag
                        sensor_bus.master_state = I2C_REPEATED_START;
                    }
                    else
                    {
                    	sensor_bus.master_state = DATA_NACK;
                    }
                }
            }
            else
            {
                if(sensor_bus.read_length != 0)
                {
                    LPC_I2C1->I2CONSET = I2CONSET_STA; // Set Repeated-start flag
                    sensor_bus.master_state = I2C_REPEATED_START;
                }
                else
                {
                    sensor_bus.master_state = DATA_NACK;
                    LPC_I2C1->I2CONSET = I2CONSET_STO; // Set Stop flag
                }
            }
            LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
            break;
        case MR_DATA_ACK:
            LPC_I2C1->I2CONSET = I2CONSET_AA; // assert ACK after data is received
            LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
            break;
        case MR_RECIEVE_ACK:
        case MR_RECIEVE_NACK:
            sensor_bus.master_buffer[sensor_bus.write_length + sensor_bus.read_index] = LPC_I2C1->I2DAT;
            sensor_bus.read_index++;
            if(sensor_bus.read_index < sensor_bus.read_length)
            {
                sensor_bus.master_state = DATA_ACK;
                LPC_I2C1->I2CONSET = I2CONSET_AA; // assert ACK after data is received
            }
            else
            {
                sensor_bus.read_index = 0;
                sensor_bus.master_state = DATA_NACK;
                LPC_I2C1->I2CONSET = I2CONSET_STO; // Set Stop flag
            }
            LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
            break;
        case MT_DATA_NACK:
        case MR_DATA_NACK:
            LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
            sensor_bus.master_state = DATA_NACK;
            break;
        case MT_ARB_LOST:
        default:
            LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
            break;
    }
}

void sensor_poll_loop()
{
	uint32_t sensor;
	uint32_t i;
	uint32_t addr;
	uint32_t loop_start_time;

	while (1)
	{
		loop_start_time = now;

		if (sensor_polling_mode != SENSOR_POLLING_MODE_POLL)
			continue;

		for (sensor=1; sensor<=NUM_SENSORS; sensor++)
		{
			sensor_read(sensor, SENSOR_ADDR_ADDRESS);
			if (sensor_memory[sensor][SENSOR_ADDR_ADDRESS]>>1 != sensor)
				continue;

			sensor_read(sensor, SENSOR_ADDR_TYPE);

			sensor_read(sensor, SENSOR_ADDR_NUM_READ_ENDPOINTS);
			sensor_read(sensor, SENSOR_ADDR_START_READ_ENDPOINTS);
			for (i=0; i<sensor_memory[sensor][SENSOR_ADDR_NUM_READ_ENDPOINTS]; i++)
			{
				sensor_read(sensor, sensor_memory[sensor][SENSOR_ADDR_START_READ_ENDPOINTS]+i);
			}

			sensor_read(sensor, SENSOR_ADDR_NUM_WRITE_ENDPOINTS);
			sensor_read(sensor, SENSOR_ADDR_START_WRITE_ENDPOINTS);
			for (i=0; i<sensor_memory[sensor][SENSOR_ADDR_NUM_WRITE_ENDPOINTS]; i++)
			{
				addr = sensor_memory[sensor][SENSOR_ADDR_START_WRITE_ENDPOINTS]+i;
				sensor_write(sensor, addr, sensor_memory[sensor][addr]);
			}
		}

		// sleep until next poll
		while (now - loop_start_time < sensor_polling_rate)
			continue;
	}
}

void sensor_set_polling_mode(uint32_t mode)
{
	sensor_polling_mode = mode;
}
void sensor_set_polling_rate(uint32_t rate)
{
	sensor_polling_rate = rate;
}

uint32_t sensor_get_polling_mode()
{
	return sensor_polling_mode;
}
uint32_t sensor_get_polling_rate()
{
	return sensor_polling_rate;
}

void sensor_set_value(uint32_t sensor, uint32_t reg, uint32_t val)
{
	if (sensor_polling_mode == SENSOR_POLLING_MODE_POLL)
		sensor_memory[sensor][reg] = val;
	else
		sensor_write(sensor, reg, val);
}

uint32_t sensor_get_value(uint32_t sensor, uint32_t reg)
{
	if (sensor_polling_mode == SENSOR_POLLING_MODE_POLL)
		return sensor_memory[sensor][reg];
	else
		return sensor_read(sensor, reg);
}
