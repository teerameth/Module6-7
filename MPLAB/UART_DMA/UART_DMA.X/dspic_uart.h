/* 
 * File:   dspic_uart.h
 * Author: Luck
 *
 * Created on August 19, 2020, 10:46 PM
 */

#ifndef DSPIC_UART_H
#define	DSPIC_UART_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

void OpenUART1(void);
unsigned char UART1_sendStr(unsigned char *pData, unsigned char numBytes );
char UART1_sendStr_INT(unsigned char *pData, unsigned char numBytes );

#endif	/* DSPIC_UART_H */

