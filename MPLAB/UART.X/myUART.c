/*
 * File:   myUART.c
 * Author: narongsak
 *
 * Created on September 3, 2020, 3:53 PM
 */

#include "xc.h"
#include "configuration.h"

volatile unsigned char data;
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void ){ 
    IFS0bits.U1TXIF = 0;
}
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void ){
    data = U1RXREG;
    U1TXREG = data;
    IFS0bits.U1RXIF = 0;
}
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1ErrInterrupt( void ){
    U1STAbits.OERR == 1 ? U1STAbits.OERR = 0 : Nop();
    IFS4bits.U1EIF = 0;
}

void initPLL() // Set Fcy to 40 MHz
{
    PLLFBD = 150;           // M  = 152
    CLKDIVbits.PLLPRE = 5;  // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0;             // Tune FRC oscillator, if FRC is used
    
    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01);    // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01);    // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK!=1) {};    // Wait for PLL to lock
}


int main(void) {
    /*disable global interrupt*/
    __builtin_disable_interrupts();
    
    initPLL();
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    RPINR18bits.U1RXR = 6;
    RPOR2bits.RP5R = 0b00011;
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
    
    U1MODEbits.STSEL = 0;   // 1 Stop bit
    U1MODEbits.PDSEL = 0;   // No Parity, 8 data bits
    U1MODEbits.BRGH = 1;    // High Speed mode
    U1MODEbits.URXINV = 0;  // UxRX idle state is '1'
    U1BRG = 86;             // BAUD Rate Setting for 115200
    U1STAbits.UTXISEL0 = 0; // Interrupt after one TX Character is transmitted
    U1STAbits.UTXISEL1 = 0;
    IEC0bits.U1RXIE = 1;    // Enable UART RX Interrupt
    IEC0bits.U1TXIE = 1;    // Enable UART TX Interrupt
    U1MODEbits.UARTEN = 1;  // Enable UART
    U1STAbits.UTXEN = 1;    // Enable UART TX
    
    /* wait at least 347 usec (1/115200) before sending first char */
    unsigned int i;
    for(i = 0; i < 347; i++){
        Nop();
    }
    /*enable global interrupt*/
    __builtin_enable_interrupts();
    
    U1TXREG = 'a'; // Transmit one character
    while(1){
    }
    return 0;
}
