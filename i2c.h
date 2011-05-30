#ifndef __I2C_H
#define __I2C_H
#define PCI2C               19
#define I2C_BUFSIZE         0x20
#define MAX_TIMEOUT         0x00FFFFFF

#define I2CMASTER           0x01
#define I2CSLAVE            0x02

#define MODULE_BUS 0
#define SENSOR_BUS 1
#define EEPROM_BUS 2

typedef struct
{
    uint32_t master_state;
    uint32_t slave_state;
    uint32_t command;
    uint32_t mode;
    uint8_t  master_buffer[I2C_BUFSIZE];
    uint8_t  slave_buffer[I2C_BUFSIZE];
    uint32_t count;
    uint32_t read_length;
    uint32_t write_length;
    uint32_t read_index;
    uint32_t write_index;
} i2c_bus_t;

/* For more info, read Philips's LM95 datasheet */
#define LM75_ADDR           0x91
#define LM75_TEMP           0x00
#define LM75_CONFIG         0x01
#define LM75_THYST          0x02
#define LM75_TOS            0x03

#define READ_BIT            0x01

#define I2C_IDLE            0
#define I2C_STARTED         1
#define I2C_RESTARTED       2
#define I2C_REPEATED_START  3
#define DATA_ACK            4
#define DATA_NACK           5

// table 398->401
// Master transmitter
#define MT_START            0x08
#define MT_REPEAT_START     0x10
#define MT_DATA_ACK         0x18
#define MT_DATA_NACK        0x20
#define MT_TRANSMIT_ACK     0x28
#define MT_TRANSMIT_NACK    0x30
#define MT_ARB_LOST         0x38
#define MR_DATA_ACK         0x40
#define MR_DATA_NACK        0x48
#define MR_RECIEVE_ACK      0x50
#define MR_RECIEVE_NACK     0x58
#define SR_ADDRESSED        0x60 // Slave has been addressed
#define SR_GEN_CALL         0x70
#define SR_DATA_RECV_ACK    0x80
#define SR_GEN_CALL_DATA    0x90
#define SR_STOP             0xA0

#define I2CONSET_I2EN       0x00000040  /* I2C Control Set Register */
#define I2CONSET_AA         0x00000004
#define I2CONSET_SI         0x00000008
#define I2CONSET_STO        0x00000010
#define I2CONSET_STA        0x00000020

#define I2CONCLR_AAC        0x00000004  /* I2C Control clear Register */
#define I2CONCLR_SIC        0x00000008
#define I2CONCLR_STAC       0x00000020
#define I2CONCLR_I2ENC      0x00000040

#define I2DAT_I2C           0x00000000  /* I2C Data Reg */
#define I2ADR_I2C           0x00000000  /* I2C Slave Address Reg */
#define I2SCLH_SCLH         0x00000080  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL         0x00000080  /* I2C SCL Duty Cycle Low Reg */

extern void I2C0_IRQHandler(void);
extern void I2C1_IRQHandler(void);
extern void I2C2_IRQHandler(void);
extern void I2cInit(uint8_t module);
extern uint32_t I2cStart(uint8_t module);
extern uint32_t I2cStop(uint8_t module);
extern uint32_t I2cEngine(uint8_t module);
extern uint32_t I2cWrite(uint8_t module, uint32_t address, uint32_t data[],
        uint32_t size);
extern uint32_t I2cRead(uint8_t module, uint32_t address, uint32_t size);

#endif
