#include "global.h"
#include "i2c.h"
#include "motor.h"

#include "consoleprint.h"

volatile i2c_bus_t i2c_bus[3];

void I2cInit(uint8_t module)
{
    // enable I2C peripheral block
    LPC_SC->PCONP |= _BIT(PCI2C);

    // config IO and peripherals
    switch(module)
    {
        case EEPROM_BUS: // I2C2 - EEPROM - I am master
            set_gpio_select(I2C_EEPROM_CLK, 2);
            set_gpio_select(I2C_EEPROM_DATA, 2);
            // Clear flags
            LPC_I2C2->I2CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;
            // Reset registers
            LPC_I2C2->I2SCLL = I2SCLL_SCLL;
            LPC_I2C2->I2SCLH = I2SCLH_SCLH;
            // Install interrupt handler
            NVIC_EnableIRQ(I2C2_IRQn);

            LPC_I2C2->I2CONSET = I2CONSET_I2EN;
            break;
    }

    // init variables
    i2c_bus[module].master_state = I2C_IDLE;
    i2c_bus[module].slave_state = I2C_IDLE;
    i2c_bus[module].count = 0;
    i2c_bus[module].read_index = 0;
    i2c_bus[module].write_index = 0;
}

uint32_t I2cStart(uint8_t module)
{
    uint32_t timeout = 0;
    uint32_t retVal = FALSE;

    // send start
    switch(module)
    {
        case EEPROM_BUS:
            LPC_I2C2->I2CONSET = I2CONSET_STA; // Set Start flag
            break;
    }

    while(1)
    {
        if(i2c_bus[module].master_state == I2C_STARTED)
        {
            retVal = TRUE;
            break;
        }
        if(timeout++ >= MAX_TIMEOUT)
        {
            retVal = FALSE;
            break;
        }
    }
    return (retVal);
}

// TODO
uint32_t I2cStop(uint8_t module)
{
    //LPC_I2C1->I2CONSET = I2CONSET_STO; /* Set Stop flag */
    //LPC_I2C1->I2CONCLR = I2CONCLR_SIC; /* Clear SI flag */

    // Wait for stop
    //while(LPC_I2C1->I2CONSET & I2CONSET_STO)
    //    ;
    return TRUE;
}

uint32_t I2cEngine(uint8_t module)
{
    i2c_bus[module].master_state = I2C_IDLE;
    i2c_bus[module].read_index = 0;
    i2c_bus[module].write_index = 0;

    if(I2cStart(module) != TRUE)
    {
        I2cStop(module);
        return (FALSE);
    }

    while(i2c_bus[module].master_state != DATA_NACK)
    {
        continue;
    }
    I2cStop(module);

    return (TRUE);
}

uint32_t I2cWrite(uint8_t module, uint32_t address, uint32_t data[], uint32_t size)
{
    uint32_t i = 0;

    i2c_bus[module].write_length = 1+size;
    i2c_bus[module].read_length = 0;

    // stuff the address as first byte sent
    i2c_bus[module].master_buffer[0] = address;
    for(i = 1;i < I2C_BUFSIZE;i++)
    {
        i2c_bus[module].master_buffer[i] = (i-1 < size) ? data[i - 1] : 0;
    }

    i2c_bus[module].command = 0;
    I2cEngine(module);
    return 0;
}

uint32_t I2cRead(uint8_t module, uint32_t address, uint32_t size)
{
    uint32_t i = 0;
    i2c_bus[module].write_length = 1; // address
    i2c_bus[module].read_length = size;

    i2c_bus[module].master_buffer[0] = address | READ_BIT;
    for(i = 1;i < I2C_BUFSIZE;i++)
    {
        i2c_bus[module].master_buffer[i] = 0;
    }

    i2c_bus[module].command = 1;
    I2cEngine(module);

    //char buffer[30];
    //snprintf(buffer, 30, "%x %d %d %d %d %d %d %d %d\n", i2c_bus[module].master_buffer[0],
    //                                         i2c_bus[module].master_buffer[1],
    //                                         i2c_bus[module].master_buffer[2],
    //                                         i2c_bus[module].master_buffer[3],
    //                                         i2c_bus[module].master_buffer[4],
    //                                         i2c_bus[module].master_buffer[5],
    //                                         i2c_bus[module].master_buffer[6],
    //                                         i2c_bus[module].master_buffer[7],
    //                                         i2c_bus[module].master_buffer[8]);
    //consoleprint(buffer);
    return i2c_bus[module].master_buffer[4];
}

void I2C2_IRQHandler(void) // EEPROM
{
    switch(LPC_I2C2->I2STAT)
    {
        case MT_START:
            LPC_I2C2->I2DAT = i2c_bus[EEPROM_BUS].master_buffer[0];
            LPC_I2C2->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
            i2c_bus[EEPROM_BUS].master_state = I2C_STARTED;
            break;
        case MT_REPEAT_START:
            if(i2c_bus[EEPROM_BUS].command == LM75_TEMP)
            {
                LPC_I2C2->I2DAT = i2c_bus[EEPROM_BUS].master_buffer[2];
            }
            LPC_I2C2->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
            i2c_bus[EEPROM_BUS].master_state = I2C_RESTARTED;
            break;
        case MT_DATA_ACK:
            if(i2c_bus[EEPROM_BUS].master_state == I2C_STARTED)
            {
                LPC_I2C2->I2DAT = i2c_bus[EEPROM_BUS].master_buffer[1 + i2c_bus[EEPROM_BUS].write_index];
                i2c_bus[EEPROM_BUS].write_index++;
                i2c_bus[EEPROM_BUS].master_state = DATA_ACK;
            }
            LPC_I2C2->I2CONCLR = I2CONCLR_SIC;
            break;
        case MT_TRANSMIT_ACK:
        case MT_TRANSMIT_NACK:
            if(i2c_bus[EEPROM_BUS].write_index
                    != i2c_bus[EEPROM_BUS].write_length)
            {
                // this should be the last one
                LPC_I2C2->I2DAT = i2c_bus[EEPROM_BUS].master_buffer[1 + i2c_bus[EEPROM_BUS].write_index];
                i2c_bus[EEPROM_BUS].write_index++;
                if(i2c_bus[EEPROM_BUS].write_index != i2c_bus[EEPROM_BUS].write_length)
                {
                    i2c_bus[EEPROM_BUS].master_state = DATA_ACK;
                }
                else
                {
                    i2c_bus[EEPROM_BUS].master_state = DATA_NACK;
                    if(i2c_bus[EEPROM_BUS].read_length != 0)
                    {
                        LPC_I2C2->I2CONSET = I2CONSET_STA; // Set Repeated-start flag
                        i2c_bus[EEPROM_BUS].master_state = I2C_REPEATED_START;
                    }
                }
            }
            else
            {
                if(i2c_bus[EEPROM_BUS].read_length != 0)
                {
                    LPC_I2C2->I2CONSET = I2CONSET_STA; // Set Repeated-start flag
                    i2c_bus[EEPROM_BUS].master_state = I2C_REPEATED_START;
                }
                else
                {
                    i2c_bus[EEPROM_BUS].master_state = DATA_NACK;
                    LPC_I2C2->I2CONSET = I2CONSET_STO; // Set Stop flag
                }
            }
            LPC_I2C2->I2CONCLR = I2CONCLR_SIC;
            break;
        case MR_DATA_ACK:
            LPC_I2C2->I2CONSET = I2CONSET_AA; // assert ACK after data is received
            LPC_I2C2->I2CONCLR = I2CONCLR_SIC;
            break;
        case MR_RECIEVE_ACK:
        case MR_RECIEVE_NACK:
            i2c_bus[EEPROM_BUS].master_buffer[3 + i2c_bus[EEPROM_BUS].read_length] = LPC_I2C2->I2DAT;
            i2c_bus[EEPROM_BUS].read_index++;
            if(i2c_bus[EEPROM_BUS].read_index != i2c_bus[EEPROM_BUS].read_length)
            {
                i2c_bus[EEPROM_BUS].master_state = DATA_ACK;
            }
            else
            {
                i2c_bus[EEPROM_BUS].read_index = 0;
                i2c_bus[EEPROM_BUS].master_state = DATA_NACK;
            }
            LPC_I2C2->I2CONSET = I2CONSET_AA; // assert ACK after data is received
            LPC_I2C2->I2CONCLR = I2CONCLR_SIC;
            break;
        case MT_DATA_NACK:
        case MR_DATA_NACK:
            LPC_I2C2->I2CONCLR = I2CONCLR_SIC;
            i2c_bus[EEPROM_BUS].master_state = DATA_NACK;
            break;
        case MT_ARB_LOST:
        default:
            LPC_I2C2->I2CONCLR = I2CONCLR_SIC;
            break;
    }
}
