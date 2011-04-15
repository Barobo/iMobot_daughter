#include "global.h"
#include "uart.h"

volatile uint32_t UART1Status;
volatile uint8_t UART1TxEmpty = 1;
volatile uint8_t UART1Buffer[BUFSIZE];
volatile uint32_t UART1Count = 0;

void UART1_IRQHandler(void)
{
    uint8_t IIRValue, LSRValue;
    uint8_t Dummy = Dummy;

    IIRValue = LPC_UART1->IIR;

    IIRValue >>= 1; // skip pending bit in IIR
    IIRValue &= 0x07; // check bit 1~3, interrupt identification
    if(IIRValue == IIR_RLS)// Receive Line Status
    {
        LSRValue = LPC_UART1->LSR;
        // Receive Line Status
        if(LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI))
        {
            // There are errors or break interrupt
            // Read LSR will clear the interrupt
            UART1Status = LSRValue;
            Dummy = LPC_UART1->RBR; // Dummy read on RX to clear interrupt, then bail out
            return;
        }
        if(LSRValue & LSR_RDR) // Receive Data Ready
        {
            // If no error on RLS, normal ready, save into the data buffer.
            // Note: read RBR will clear the interrupt
            UART1Buffer[UART1Count] = LPC_UART1->RBR;
            UART1Count++;
            if(UART1Count == BUFSIZE)
            {
                UART1Count = 0; // buffer overflow
            }
        }
    }
    else if(IIRValue == IIR_RDA) // Receive Data Available
    {
        // Receive Data Available
        UART1Buffer[UART1Count] = LPC_UART1->RBR;
        UART1Count++;
        if(UART1Count == BUFSIZE)
        {
            UART1Count = 0; // buffer overflow
        }
    }
    else if(IIRValue == IIR_CTI) // Character timeout indicator
    {
        // Character Time-out indicator
        UART1Status |= 0x100; // Bit 9 as the CTI error
    }
    else if(IIRValue == IIR_THRE) // THRE, transmit holding register empty
    {
        // THRE interrupt
        LSRValue = LPC_UART1->LSR; // Check status in the LSR to see if valid data in U0THR or not
        if(LSRValue & LSR_THRE)
        {
            UART1TxEmpty = 1;
        }
    }
}

uint32_t UartInit(uint32_t baudrate)
{
    uint32_t Fdiv;
    uint32_t pclkdiv, pclk;

    // config the lines for uart page 108
    set_gpio_pin(CONSOLE_TX, 1);
    set_gpio_pin(CONSOLE_RX, 1);

    // By default, the PCLKSELx value is zero, thus, the PCLK for
    // all the peripherals is 1/4 of the SystemFrequency.
    // Bit 8,9 are for UART1
    pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
    switch(pclkdiv)
    {
        case 0x00:
        default:
            pclk = SystemCoreClock / 4;
            break;
        case 0x01:
            pclk = SystemCoreClock;
            break;
        case 0x02:
            pclk = SystemCoreClock / 2;
            break;
        case 0x03:
            pclk = SystemCoreClock / 8;
            break;
    }

    LPC_UART1->LCR = 0x83; /* 8 bits, no Parity, 1 Stop bit */
    Fdiv = (pclk / 16) / baudrate; /*baud rate */
    LPC_UART1->DLM = Fdiv / 256;
    LPC_UART1->DLL = Fdiv % 256;
    LPC_UART1->LCR = 0x03; /* DLAB = 0 */
    LPC_UART1->FCR = 0x07; /* Enable and reset TX and RX FIFO. */

    NVIC_EnableIRQ(UART1_IRQn);

    LPC_UART1->IER = IER_RBR | IER_THRE | IER_RLS; /* Enable UART1 interrupt */
    return (TRUE);
}

void UartSend(uint8_t *BufferPtr, uint32_t Length)
{
    while(Length != 0)
    {
        // THRE status, contain valid data
        while(!(UART1TxEmpty & 0x01))
            ;
        LPC_UART1->THR = *BufferPtr;
        UART1TxEmpty = 0; /* not empty in the THR until it shifts out */
        BufferPtr++;
        Length--;
    }
}
