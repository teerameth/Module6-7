/* 
 * File:   uart_dma_main.c
 * Author: Luck
 *
 * Created on October 28, 2020, 11:34 PM
 */


#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include "configuration.h"


/*
 * 
 */



#define FCY = 40000000
volatile uint64_t millis = 0;
volatile unsigned char rec_char = '2';
unsigned char str_buff[100];
//holds the char sent
extern volatile char uart1_rcvd_char;
//flag indicating char has been rcvd
extern volatile char uart1_rcvd;


void initGPIO()
{
    AD1PCFGL = 0xFFFF;          //set analog input to digital pin
    TRISBbits.TRISB2 = 0; //set RB2 to output
    TRISBbits.TRISB3 = 0; //set RB3 to output
    
    TRISAbits.TRISA0 =0;
    
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

	//This routine Configures DMAchannel 1 for reception.
	cfgDma1UartRx();

	// UART Configurations
     cfgUart1();
     
    T1CONbits.TON =1;            //enable timer1
    
    while(1){
        if (millis > 1000){
            millis = 0;
            //printf("1 sec\n");
            LATBbits.LATB2 ^= 1;            //toggle RB0
            
            LATAbits.LATA0 ^=1;
            
        }
    }
    
    return (EXIT_SUCCESS);
}

