/* 
 * File:   uart_dma_main.c
 * Author: Luck
 *
 * Created on October 28, 2020, 11:34 PM
 */


#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <p33FJ64MC802.h>
#include "configuration.h"



/*
 * 
 */



#define FCY = 40008571
#define BAUDRATE 19200                
#define BRGVAL   (((FCY/BAUDRATE)/16)-1 )

volatile uint64_t millis = 0;
volatile unsigned char rec_char = '2';
unsigned char str_buff[100];
//holds the char sent
extern volatile char uart1_rcvd_char;
//flag indicating char has been rcvd
extern volatile char uart1_rcvd;

unsigned char BufferA[100]  __attribute__((space(dma)));

unsigned int count;

volatile unsigned char dma_ing = 0;;



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
	//  STEP 5:
	//  Configure DMA Channel 0 to:
	//  Transfer data from RAM to UART
	//  One-Shot mode
	//  Register Indirect with Post-Increment
	//  Using single buffer
	//  8 transfers per buffer
	//  Transfer words
	//********************************************************************************/
	//DMA0CON = 0x2001;					// One-Shot, Post-Increment, RAM-to-Peripheral
	DMA0CONbits.AMODE = 0;
	DMA0CONbits.MODE  = 1;
	DMA0CONbits.DIR   = 1;
	DMA0CONbits.SIZE  = 1;
	DMA0CNT = 7;						// 8 DMA requests

	//********************************************************************************
	//  STEP 6:
	// Associate one buffer with Channel 0 for one-shot operation
	//********************************************************************************/
	DMA0STA = __builtin_dmaoffset(BufferA);

	//********************************************************************************
	//  STEP 8:
	//	Enable DMA Interrupts
	//********************************************************************************/
	IFS0bits.DMA0IF  = 0;			// Clear DMA Interrupt Flag
	IEC0bits.DMA0IE  = 1;			// Enable DMA interrupt
    
    dma_ing = 0;

}

void cfgUart1(void)
{
	U1MODEbits.STSEL = 0;			// 1-stop bit
	U1MODEbits.PDSEL = 0;			// No Parity, 8-data bits
	U1MODEbits.ABAUD = 0;			// Autobaud Disabled

	U1BRG = 129;					// BAUD Rate Setting for 9600


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

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
	IFS0bits.DMA0IF = 0;			// Clear the DMA0 Interrupt Flag;
    dma_ing = 0;
}

void initGPIO()
{
    AD1PCFGL = 0xFFFF;          //set analog input to digital pin
    //TRISBbits.TRISB0 = 0; //set RB0 to output
    //TRISBbits.TRISB1 = 0; //set RB0 to output
//    TRISBbits.TRISB2 = 0; //set RB2 to output
//    TRISBbits.TRISB3 = 0; //set RB3 to output
    AD1PCFGLbits.PCFG0 = 0;		// AN0 as Analog Input
    
    TRISAbits.TRISA4 = 0;
    
}


//interrupt function
void __attribute__((interrupt,no_auto_psv)) _T1Interrupt(void)
{
    millis++;   //add millis by 1 every 1 ms
    _T1IF =0 ;  //clear interrupt flag
}


void initPLL()
{
    PLLFBD = 63;           // M  = 152
    CLKDIVbits.PLLPRE = 1;  // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0;             // Tune FRC oscillator, if FRC is used
    
    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01);    // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01);    // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK!=1) {};    // Wait for PLL to lock
}



int main(int argc, char** argv) {

    
    __builtin_disable_interrupts();
    
    initPLL();
    
    initGPIO();
    
    AD1CON1bits.AD12B = 0;
    AD1CON3bits.ADCS = 2;
    AD1CON1bits.ADON = 1;
    
    __builtin_write_OSCCONL(OSCCON & 0xBF); // to clear IOLOCK
    
    
         // Assign U1TX to RP6, Pin 15
    RPOR2bits.RP5R = 3;
    // Assign U1RX to RP5, Pin 14
    RPINR18bits.U1RXR = 6;
    
    __builtin_write_OSCCONL(OSCCON | 0x40); // to set IOLOCK
    
    
    T1CONbits.TCKPS = 0b01; //set timer prescaler to 1:8
    PR1 = 5000;             //set period to 1 ms
    _T1IE = 1;              //enable interrupt for timer1
    _T1IP = 3;              //set interrupt priority to 3 
    
    //OpenUART1();
    
   
    
     /*enable global interrupt*/
    __builtin_enable_interrupts();
    
    //This routine Configures DMAchannel 0 for transmission.	
	cfgDma0UartTx();


	// UART Configurations
     cfgUart1();
     
    T1CONbits.TON =1;            //enable timer1
    count = 0;
    while(1){
        if (millis > 100){
            int i;
            millis = 0;
            
            LATAbits.LATA4 ^= 1; 
            
            AD1CON1bits.SAMP = 1;
            for(i=0;i<15;i++);
            AD1CON1bits.SAMP = 0;
            while(!AD1CON1bits.DONE);
            AD1CON1bits.DONE = 0;

            
            if(dma_ing==0)
            {
                sprintf(BufferA,"\rThis is a test for uart1 loop no: %d \r", ADC1BUF0); 
                DMA0CNT = strlen(BufferA)-1;
                DMA0STA = __builtin_dmaoffset(BufferA);
                
                dma_ing = 1;

                DMA0CONbits.CHEN  = 1;			// Re-enable DMA0 Channel
                DMA0REQbits.FORCE = 1;			// Manual mode: Kick-start the first transfer
            }
                        
            count++;
            
        }
    }
    
    return (EXIT_SUCCESS);
}

