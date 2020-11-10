/*
 * File:   uart1.c
 * Author: G_Peav
 *
 * Created on September 9, 2019, 4:31 PM
 */


#include "xc.h"
#include "configuration.h"
#include "UART_rev3.h"

int numByte;
uint8_t dataArray[10], stack[20];
uint8_t stackSize=0, startIndex, packetLength, uart_state = 0, checksum;
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
void shift_buffer(int n, uint8_t *buffer, int buffer_size){ // Shift buffer to the left N times
    int i, j;
    for(i=0;i<n;i++){
        for(j=0;j<buffer_size-1;j++){
            buffer[j] = buffer[j+1];
        }
    }
}
void readPosition(){
    
}
void writePosition(int x, int y){
    // Acknowledge
    // Move
    // Report
}
void writeTrajectory(int a, int b, int c, int d, int e, int f, int g, int h){
    // Acknowledge
    // Move
    // Report
}
void setHome(){
    // Acknowledge
    // Move
    // Report
}
void askHome(){
    // Acknowledge
    // Move
    // Report
}
void circularMotion(int n){
    
}
int main(void) {
    /*disable global interrupt*/
    __builtin_disable_interrupts();
    
    initPLL();
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    RPINR18bits.U1RXR = 6;
    RPOR2bits.RP5R = 0b00011;
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK 
    
    UART1_Initialize(86, 347);

    /*enable global interrupt*/
    __builtin_enable_interrupts();
    
    printf("Program Start!\n");
    
    while(1){
        numByte = UART1_ReadBuffer(dataArray, 10); // numByte = Number of data taking out of buffer and put into array
        int i;
        unsigned char checksum;
        if (numByte != 0){    
            for (i=0;i<numByte;i++){
                stack[stackSize+i] = dataArray[i];
            }
            stackSize += numByte;
        }
        if (stackSize > 2) // Time to check data
        {
            switch(uart_state){
                case 0:
                    for(i=0;i<stackSize-1;i++){
                        if(stack[i]==255 && stack[i+1]==255){
                            startIndex = i;
                            if(stackSize - startIndex < 3){break;} // packetLength doesn't arrived yet
                            uart_state = 1;
                            break;
                        }
                    }
                    break;
                case 1:
                    packetLength = stack[startIndex+2];
                    printf("packetLength = %d\n", packetLength);
                    printf("stackSize = %d\n", stackSize);
                    printf("startIndex = %d\n", startIndex);
                    if(stackSize - startIndex > packetLength+2){ // Packet complete -> Move to next step
//                        printf("packetLength = %d\n", packetLength);
                        uart_state = 2;
                        break;
                    }
                    break; //transmission doesn't finished yet
                case 2: // Checksum recognize
                    checksum = 0;
                    for(i=startIndex+2;i<startIndex+packetLength+2;i++){checksum+=stack[i];}
                    checksum = ~checksum;
                    
                    if(stack[startIndex+packetLength+2] == checksum){ // Checksum correct
                        printf("Checksum Passed!\n");
                        uart_state = 3;
                        break;
                    }
                    else{
                        // Checksum error
                        printf("Checksum Error!\n");
//                        for (i=0;i<sizeof(stack);i++){printf("%d ", stack[i]);}printf("\n");
                        shift_buffer(startIndex+1, stack, sizeof(stack));
                        stackSize -= startIndex+1;
                        break;
                    }
                case 3: // Call function
                    switch(stack[startIndex+3]){ //Instruction (ParameterN is at startIndex+3+N)
                        case 1: //Ping
                            printf("Instruction 1\n");
                            break;
                        case 2: //Read(x, y)
                            printf("Instruction 2\n");
                            break;
                        case 3: //Write (x, y)
                            printf("Instruction 3\n");
                            break;
                        case 4: //Write Trajectory
                            printf("Instruction 4\n");
                            break;
                        case 5: // Home
                            printf("Instruction 5\n");
                            break;
                        case 7: // Motion
                            printf("Instruction 7\n");
                            break;
                        default:
                            printf("Unknown Instruction\n");
                    }
                    printf("stackSize = %d\n", stackSize);
                    for(i=0;i<sizeof(stack);i++){printf("%d ", stack[i]);}printf("\n");
                    shift_buffer(startIndex+packetLength+3, &stack, sizeof(stack));//Clear used stack
                    stackSize -= startIndex+packetLength+3;
                    for(i=0;i<sizeof(stack);i++){printf("%d ", stack[i]);}printf("\n");
                    printf("stackSize = %d\n", stackSize);
                    uart_state = 0;
            }
        }
    }
    return 0;
}