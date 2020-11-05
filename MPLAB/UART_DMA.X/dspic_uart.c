#include "xc.h"
#include "dspic_uart.h"


//holds the char sent
volatile char uart1_rcvd_char;
//flag indicating char has been rcvd
volatile char uart1_rcvd;

volatile char int_sending;
volatile char nums_sending_byte;
volatile unsigned char send_buff[100];
volatile unsigned char byte_sent;

void OpenUART1(void)
{
    
    int_sending = 0;
    nums_sending_byte = 0;
    byte_sent = 0;
    
	U1MODEbits.UARTEN = 0;	// Bit15 TX, RX DISABLED, ENABLE at end of func
	//U2MODEbits.notimplemented;	// Bit14
	U1MODEbits.USIDL = 0;	// Bit13 Continue in Idle
	U1MODEbits.IREN = 0;	// Bit12 No IR translation
	U1MODEbits.RTSMD = 0;	// Bit11 Simplex Mode
	//U2MODEbits.notimplemented;	// Bit10
	U1MODEbits.UEN = 0;		// Bits8,9 TX,RX enabled, CTS,RTS not
	U1MODEbits.WAKE = 0;	// Bit7 No Wake up (since we don't sleep here)
	U1MODEbits.LPBACK = 0;	// Bit6 No Loop Back
	U1MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
	U1MODEbits.URXINV = 0;	// Bit4 IdleState = 1  (for dsPIC)
	U1MODEbits.BRGH = 0;	// Bit3 16 clocks per bit period
	U1MODEbits.PDSEL = 0;	// Bits1,2 8bit, No Parity
	U1MODEbits.STSEL = 0;	// Bit0 One Stop Bit
    
    U1BRG=129;
    
   // Load all values in for U1STA SFR
	U1STAbits.UTXISEL1 = 0;	//Bit15 Int when Char is transferred (1/2 config!)
	U1STAbits.UTXINV = 0;	//Bit14 N/A, IRDA config
	U1STAbits.UTXISEL0 = 0;	//Bit13 Other half of Bit15
	//U2STAbits.notimplemented = 0;	//Bit12
	U1STAbits.UTXBRK = 0;	//Bit11 Disabled
	U1STAbits.UTXEN = 0;	//Bit10 TX pins controlled by periph
	U1STAbits.UTXBF = 0;	//Bit9 *Read Only Bit*
	U1STAbits.TRMT = 0;	//Bit8 *Read Only bit*
	U1STAbits.URXISEL = 0;	//Bits6,7 Int. on character recieved
	U1STAbits.ADDEN = 0;	//Bit5 Address Detect Disabled
	U1STAbits.RIDLE = 0;	//Bit4 *Read Only Bit*
	U1STAbits.PERR = 0;		//Bit3 *Read Only Bit*
	U1STAbits.FERR = 0;		//Bit2 *Read Only Bit*
	U1STAbits.OERR = 0;		//Bit1 *Read Only Bit*
	U1STAbits.URXDA = 0;	//Bit0 *Read Only Bit*

    //interrupt after one character is rcvd
    U1STAbits.URXISEL = 0;
    
    //clear flag then enable interrupts
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    
	U1MODEbits.UARTEN = 1; //enable uart
    U1STAbits.UTXEN = 1; //transmitter enabled
    
    
}

unsigned char UART1_sendStr(unsigned char *pData, unsigned char numBytes )
{
	unsigned char numWrites = 0;
	
	while(numBytes)
    {
        U1TXREG = *pData;
        while(U1STAbits.TRMT == 0);
        pData++;
        numBytes--;
    }

	return numWrites;
}	

char UART1_sendStr_INT(unsigned char *pData, unsigned char numBytes )
{
    
    if(int_sending == 0)
    {
        int_sending = 1;
        IFS0bits.U1TXIF = 0;
        IEC0bits.U1TXIE = 1; // Enable UART TX interrupt
        nums_sending_byte = numBytes;
        memcpy(send_buff,pData,numBytes);
        U1TXREG = send_buff[byte_sent]; // Transmit first byte
        byte_sent++;
        nums_sending_byte--;
    }
    else
        return -1;
    
    return 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{

	// Clear interrupt flag
	IFS0bits.U1RXIF = 0;
    //let the main loop know we received a char
    uart1_rcvd = 1;
    //load the char
    uart1_rcvd_char = U1RXREG;
    
//    LATBbits.LATB3 ^= 1;            //toggle RB1
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    
    if(nums_sending_byte>1)
    {
        U1TXREG = send_buff[byte_sent]; // Transmit first byte
        byte_sent++;
        nums_sending_byte--;
        LATBbits.LATB3 ^= 1;            //toggle RB1
    }
    else
    {
        IEC0bits.U1TXIE = 0; // Disable UART TX interrupt
        int_sending = 0;
        nums_sending_byte = 0;
        byte_sent = 0;
    }
}