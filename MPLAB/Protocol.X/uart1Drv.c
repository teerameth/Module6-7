#include "xc.h"

#define FCY      40000000
#define BAUDRATE 115200             
#define BRGVAL   ((FCY/BAUDRATE)/16)-1 

//********************************************************************************
//  STEP 6:
//  Allocate two buffers for DMA transfers
//********************************************************************************/
unsigned int BufferA[20] __attribute__((space(dma)));
unsigned int BufferB[20] __attribute__((space(dma)));


// UART Configuration
void cfgUart1(void)
{
	U1MODEbits.STSEL = 0;			// 1-stop bit
	U1MODEbits.PDSEL = 0;			// No Parity, 8-data bits
	U1MODEbits.ABAUD = 0;			// Autobaud Disabled
	U1BRG = BRGVAL;					// BAUD Rate Setting for 9600
	//********************************************************************************
	//  STEP 1:
	//  Configure UART for DMA transfers
	//********************************************************************************/
	U1STAbits.UTXISEL0 = 0;			// Interrupt after one Tx character is transmitted
	U1STAbits.UTXISEL1 = 0;			                            
	U1STAbits.URXISEL  = 0;			// Interrupt after one RX character is received
	//********************************************************************************
	//  STEP 2:
	//  Enable UART Rx and Tx
	//********************************************************************************/
	U1MODEbits.UARTEN   = 1;		// Enable UART
	U1STAbits.UTXEN     = 1;		// Enable UART Tx
	IEC4bits.U1EIE = 0;
}
// DMA0 configuration
void cfgDma0UartTx(void)
{
	//********************************************************************************
	//  STEP 3:
	//  Associate DMA Channel 0 with UART Tx
	//********************************************************************************/
	DMA0REQ = 0x000C;					// Select UART1 Transmitter
	DMA0PAD = (volatile unsigned int) &U1TXREG;
	
	//********************************************************************************
	//  STEP 5: Configure DMA Channel 0 to: Transfer data from RAM to UART One-Shot mode Register Indirect with Post-Increment Using single buffer 8 transfers per buffer Transfer words
	//********************************************************************************/
	// One-Shot, Post-Increment, RAM-to-Peripheral
	DMA0CONbits.AMODE = 0;
	DMA0CONbits.MODE  = 1;
	DMA0CONbits.DIR   = 1;
	DMA0CONbits.SIZE  = 0;
	DMA0CNT = 3;						// TX Count
	//********************************************************************************
	//  STEP 6: Associate one buffer with Channel 0 for one-shot operation
	//********************************************************************************/
	DMA0STA = __builtin_dmaoffset(BufferB);

	//********************************************************************************
	//  STEP 8: Enable DMA Interrupts
	//********************************************************************************/
	IFS0bits.DMA0IF  = 0;			// Clear DMA Interrupt Flag
	IEC0bits.DMA0IE  = 1;			// Enable DMA interrupt
}

// DMA1 configuration
void cfgDma1UartRx(void)
{
	//********************************************************************************
	//  STEP 3: Associate DMA Channel 1 with UART Rx
	//********************************************************************************/
	DMA1REQ = 0x000B;					// Select UART1 Receiver
	DMA1PAD = (volatile unsigned int) &U1RXREG;

	//********************************************************************************
	//  STEP 4: Configure DMA Channel 1 to: Transfer data from UART to RAM Continuously Register Indirect with Post-Increment Using two ‘ping-pong’ buffers 8 transfers per buffer Transfer words
	//********************************************************************************/
	//DMA1CON = 0x0002;					// Continuous,  Post-Inc, Periph-RAM
	DMA1CONbits.AMODE = 1;
	DMA1CONbits.MODE  = 0;
	DMA1CONbits.DIR   = 0;
	DMA1CONbits.SIZE  = 0;
	DMA1CNT = 5;						// RX Count

	//********************************************************************************
	//  STEP 6: Associate two buffers with Channel 1 for ‘Ping-Pong’ operation
	//********************************************************************************/
	DMA1STA = __builtin_dmaoffset(BufferA);

	//********************************************************************************
	//  STEP 8: Enable DMA Interrupts
	//********************************************************************************/
	IFS0bits.DMA1IF  = 0;			// Clear DMA interrupt
	IEC0bits.DMA1IE  = 1;			// Enable DMA interrupt

	//********************************************************************************
	//  STEP 9: Enable DMA Channel 1 to receive UART data
	//********************************************************************************/
	DMA1CONbits.CHEN = 1;			// Enable DMA Channel
}



//********************************************************************************
//  STEP 7: Setup DMA interrupt handlers Force transmit after 8 words are received
//********************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
	IFS0bits.DMA0IF = 0;			// Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{   
    DMA0CNT = 19;
    DMA0STA = __builtin_dmaoffset(BufferA); // Point DMA 0 to data to be transmitted

	DMA0CONbits.CHEN  = 1;			// Re-enable DMA0 Channel
	DMA0REQbits.FORCE = 1;			// Manual mode: Kick-start the first transfer

	IFS0bits.DMA1IF = 0;			// Clear the DMA1 Interrupt Flag
}



void __attribute__ ((interrupt, no_auto_psv)) _U1ErrInterrupt(void)
{
	IFS4bits.U1EIF = 0; // Clear the UART2 Error Interrupt Flag
}

