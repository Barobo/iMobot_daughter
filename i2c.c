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
        case MODULE_BUS: // I2C0 - Module bus - I am slave
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
            LPC_I2C0->I2ADR0 = 0x55<<1; // TODO: give address ; Shift by one
                                        // because first bit is reserved for
                                        // "general call enable". Sec 19.8.7
            LPC_I2C0->I2MASK0 = 0x7F<<1; // Address mask

            // Install interrupt handler
            NVIC_EnableIRQ(I2C0_IRQn);

            LPC_I2C0->I2CONSET = I2CONSET_I2EN | I2CONSET_AA;
            //LPC_I2C0->I2CONCLR = I2CONCLR_STAC | I2CONCLR_STOC | I2CONCLR_SIC;
            break;
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
        case MODULE_BUS:
            LPC_I2C0->I2CONSET = I2CONSET_STA; // Set Start flag
            break;
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

void I2C0_IRQHandler(void) // MODULE
{
    switch(LPC_I2C0->I2STAT)
    {
        case 0x0: /* Error */
            LPC_I2C0->I2CONSET = 0x14;
            LPC_I2C0->I2CONCLR = 0x08;
        case MT_START:
            LPC_I2C0->I2DAT = i2c_bus[MODULE_BUS].master_buffer[0];
            LPC_I2C0->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
            i2c_bus[MODULE_BUS].master_state = I2C_STARTED;
            break;
        case MT_REPEAT_START:
            if(i2c_bus[MODULE_BUS].command == LM75_TEMP)
            {
                LPC_I2C0->I2DAT = i2c_bus[MODULE_BUS].master_buffer[2];
            }
            LPC_I2C0->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
            i2c_bus[MODULE_BUS].master_state = I2C_RESTARTED;
            break;
        case MT_DATA_ACK:
            if(i2c_bus[MODULE_BUS].master_state == I2C_STARTED)
            {
                LPC_I2C0->I2DAT = i2c_bus[MODULE_BUS].master_buffer[1 + i2c_bus[MODULE_BUS].write_index];
                i2c_bus[MODULE_BUS].write_index++;
                i2c_bus[MODULE_BUS].master_state = DATA_ACK;
            }
            LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
            break;
        case MT_TRANSMIT_ACK:
        case MT_TRANSMIT_NACK:
            if(i2c_bus[MODULE_BUS].write_index
                    != i2c_bus[MODULE_BUS].write_length)
            {
                // this should be the last one
                LPC_I2C0->I2DAT = i2c_bus[MODULE_BUS].master_buffer[1 + i2c_bus[MODULE_BUS].write_index];
                i2c_bus[MODULE_BUS].write_index++;
                if(i2c_bus[MODULE_BUS].write_index != i2c_bus[MODULE_BUS].write_length)
                {
                    i2c_bus[MODULE_BUS].master_state = DATA_ACK;
                }
                else
                {
                    i2c_bus[MODULE_BUS].master_state = DATA_NACK;
                    if(i2c_bus[MODULE_BUS].read_length != 0)
                    {
                        LPC_I2C0->I2CONSET = I2CONSET_STA; // Set Repeated-start flag
                        i2c_bus[MODULE_BUS].master_state = I2C_REPEATED_START;
                    }
                }
            }
            else
            {
                if(i2c_bus[MODULE_BUS].read_length != 0)
                {
                    LPC_I2C0->I2CONSET = I2CONSET_STA; // Set Repeated-start flag
                    i2c_bus[MODULE_BUS].master_state = I2C_REPEATED_START;
                }
                else
                {
                    i2c_bus[MODULE_BUS].master_state = DATA_NACK;
                    LPC_I2C0->I2CONSET = I2CONSET_STO; // Set Stop flag
                }
            }
            LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
            break;
        case MR_DATA_ACK:
            LPC_I2C0->I2CONSET = I2CONSET_AA; // assert ACK after data is received
            LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
            break;
        case MR_RECIEVE_ACK:
        case MR_RECIEVE_NACK:
            i2c_bus[MODULE_BUS].master_buffer[3 + i2c_bus[MODULE_BUS].read_length] = LPC_I2C0->I2DAT;
            i2c_bus[MODULE_BUS].read_index++;
            if(i2c_bus[MODULE_BUS].read_index != i2c_bus[MODULE_BUS].read_length)
            {
                i2c_bus[MODULE_BUS].master_state = DATA_ACK;
            }
            else
            {
                i2c_bus[MODULE_BUS].read_index = 0;
                i2c_bus[MODULE_BUS].master_state = DATA_NACK;
            }
            LPC_I2C0->I2CONSET = I2CONSET_AA; // assert ACK after data is received
            LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
            break;
        case MT_DATA_NACK:
        case MR_DATA_NACK:
            LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
            i2c_bus[MODULE_BUS].master_state = DATA_NACK;
            break;
        case SR_ADDRESSED:
          LPC_I2C0->I2CONSET = 0x04; 
          LPC_I2C0->I2CONCLR = 0x08;
          break;
          
        case SR_GEN_CALL:
        case SR_DATA_RECV_ACK:
        case SR_GEN_CALL_DATA:
        case SR_STOP:
        case ST_ADDRESSED:
        case ST_TRANSMIT_ACK:
        case ST_TRANSMIT_NACK:
          //slave_reset();
          //LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
          //break;
          slave_recv_state_machine(LPC_I2C0->I2STAT);
          LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
          break;
        case MT_ARB_LOST:
        default:
            LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
            break;
    }
}

enum slave_state_e {
  SLAVE_IDLE,
  SLAVE_RECV,
  SLAVE_SEND };

void slave_recv_state_machine(uint32_t I2C_state)
{
  static enum slave_state_e state = SLAVE_IDLE;
  //static int recv_len;
  //static int recv_index;
  static uint8_t reg;
  //static uint8_t recv_buffer[20];
  uint8_t dat;
  switch(I2C_state) {
    case SR_ADDRESSED: /* We have been addressed. */
      reply_ack();
      break;
    case SR_DATA_RECV_ACK:
      dat = LPC_I2C0->I2DAT;
      switch(state) {
        case SLAVE_IDLE:
          /* This is the first byte we are receiving. This byte will indicate
           * the register address to write/read to. */
          /* Check to make sure address is within bounds */
          if(dat < 0x30 || dat > 0x67) {
            reply_nack();
            break;
          }
          /* It is in bounds. Process the address and set the correct state. */
          reg = dat;
          state = SLAVE_RECV;
          reply_ack();
          break;
        case SLAVE_RECV:
          /* For now, we only receive 1 byte at a time. Multi-byte transfers
           * will be implemented later. */
          slave_write_register(reg, LPC_I2C0->I2DAT);
          state = SLAVE_IDLE;
          reply_nack();
        default:
          reply_nack();
          break;
      }
      break;
    case SR_STOP:
      reply_ack();
      break;

    /* 
     * Slave Transmitter States *
     */
    case ST_ADDRESSED:
      /* Make sure we are in the "SLAVE_RECV" state, which means a valid
       * address has been loaded into the register. */
      if(state != SLAVE_RECV) {
        reply_nack();
        state = SLAVE_IDLE;
        break;
      } else {
        slave_read_register(reg);
        state = SLAVE_IDLE;
        reply_ack();
        break;
      }
      break;
    case ST_TRANSMIT_NACK:
      reply_ack();
      state = SLAVE_IDLE;
      break;
    default:
      reply_nack();
      break;
  }
}

#define I2C_MOTOR_SPEED 20
void slave_write_register(uint8_t reg, uint8_t dat)
{
  uint8_t motor_index;
  motor_index = (reg>>4) - 3;
  /* if lower word is "2" or "3", we want to write to motor positions. */
  if( (0x0F&reg) == 0x02) {
    /* Write to the high byte of the motor position */
    /* First, clear high byte */
    motor[motor_index].desired_position &= 0x00FF;
    /* Now write it */
    motor[motor_index].desired_position |= dat<<8;
    /* If the highest bit is set, it must be a negative number. */
    if(dat & 0x80) {
      motor[motor_index].desired_position |= 0xFFFF0000;
    }
    /* If the motor direction is AUTO, start moving the motor towards the goal */
    if(motor[motor_index].direction == MOTOR_DIR_AUTO) {
      set_motor_position_abs(
          motor_index, 
          motor[motor_index].desired_position, 
          motor[motor_index].speed);
    }
  }
  if( (0x0F&reg) == 0x03) {
    /* Write to the low byte of the motor position */
    /* First, clear low byte */
    motor[motor_index].desired_position &= 0xFFFFFF00;
    /* Now write it */
    motor[motor_index].desired_position |= (uint16_t) (0x00FF & dat);
    /* If the motor direction is AUTO, start moving the motor towards the goal */
    if(motor[motor_index].direction == MOTOR_DIR_AUTO) {
      set_motor_position_abs(
          motor_index, 
          motor[motor_index].desired_position, 
          motor[motor_index].speed);
    }
  }
  if( (0x0F&reg) == 0x04) {
    /* Set the "direction" register */
    motor[motor_index].direction = dat;
    /* If the new direction is not AUTO, start turning the motor in that
     * direction at the motor speed. */
    if(motor[motor_index].direction == MOTOR_DIR_FORWARD) {
      set_motor_speed(motor_index, motor[motor_index].speed);
    } else if (motor[motor_index].direction == MOTOR_DIR_BACKWARD) {
      set_motor_speed(motor_index, -1*motor[motor_index].speed);
    } else {
      set_motor_position_abs(
          motor_index, 
          motor[motor_index].desired_position, 
          motor[motor_index].speed);
    }
  }
  if( (0x0F&reg) == 0x05) {
    /* Set the "speed" register */
    motor[motor_index].speed = dat;
    /* If setting the motor speed to zero, stop the motor immediately */
    if(motor[motor_index].speed == 0) {
      set_motor_speed(motor_index, 0);
    }
    /* Check the direction register. If set to AUTO, do not start moving the
     * motor yet. */
    if(motor[motor_index].direction == MOTOR_DIR_FORWARD) {
      set_motor_speed(motor_index, dat);
    } else if (motor[motor_index].direction == MOTOR_DIR_BACKWARD) {
      set_motor_speed(motor_index, -dat);
    }   
  }
}

void slave_read_register(uint8_t reg)
{
  int32_t* enc = g_enc;
  uint8_t motor_index;
  motor_index = (reg>>4) - 3;
  /* if lower word is "2" or "3", we want to read motor positions. */
  if( (0x0F&reg) == 0x02) {
    /* read the high byte of the motor position */
    LPC_I2C0->I2DAT = 0x00FF & (enc[motor_index]>>8);
  }
  if( (0x0F&reg) == 0x03) {
    /* read the low byte of the motor position */
    LPC_I2C0->I2DAT = 0x00FF & (enc[motor_index]);
  }
  if( (0x0F&reg) == 0x04) {
    /* read the "direction" register */
    LPC_I2C0->I2DAT = 0x00FF & (motor[motor_index].direction);
  }
  if( (0x0F&reg) == 0x05) {
    /* read the "speed" register */
    LPC_I2C0->I2DAT = 0x00FF & (motor[motor_index].speed);
  }
  if( (0x0F&reg) == 0x07) {
    /* Read the motor status */
    LPC_I2C0->I2DAT = motor[motor_index].state;
  }
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
  LPC_I2C0->I2CONCLR = 0x0C;
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
