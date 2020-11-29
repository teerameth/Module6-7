/*
 * File:   uart.c
 * Author: teera
 *
 * Created on November 0, 2020, 5:99 AM
 */

#include "xc.h"
#include "stdio.h"
#include "configuration.h"
#include "UART_rev3.h"
#include <math.h>

#define PI 3.14159265
#define G 9.782970341
#define CON_T 0.002

float t,tf,ti,r,theta,c1,c2,c3,c4,gramma;
int state = 0, n = 1;
float x0,y0, Q_t,Q_dot_t,path_mode,phi;

int numByte;
uint8_t dataArray[10], stack[40];
unsigned int BufferA[10] __attribute__((space(dma)));
uint8_t stackSize=0, startIndex, packetLength, uart_state = 0, checksum, circular_nf;
bool circle_running = false, trajectory_running = false, homed = false;
int cur_pos_x, cur_pos_y;
uint8_t ack_packet[10] = {255, 255, 0, 0, 0, 0, 0, 0, 0, 0};

void initPLL();
void setup();
float read_posX();
float read_posY();
void readPosition();
void writePosition(float x, float y);
void writeTrajectory(float C3, float C4, float X0, float Y0, float THETA, float GRAMMA, float TF);
void applyCheckSum(uint8_t *buffer, int length);
void shift_buffer(int n, uint8_t *buffer, int buffer_size);
void sendUART(int length, uint8_t *buffer, int startIndex);
void setHome();
void circularMotion(int nf);
void recMotion(int nf);
void delay();
void sendAck();
int main(void) {
    setup();
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
                    if(stackSize - startIndex > packetLength+2){ // Packet complete -> Move to next step
                        uart_state = 2;
                        break;
                    }
                    break; //transmission doesn't finished yet
                case 2: // Checksum recognize
                    checksum = 0;
                    for(i=startIndex+2;i<startIndex+packetLength+2;i++){checksum+=stack[i];}
                    checksum = ~checksum;
                    if(stack[startIndex+packetLength+2] == checksum){ // Checksum correct
//                        printf("Checksum Passed!\n");
                        uart_state = 3;
                        break;
                    }
                    else{
                        // Checksum error
//                        printf("Checksum Error!\n");
//                            ack_packet = {255, 255, 3, 6, 1, checksum};
                        ack_packet[2] = 3;
                        ack_packet[3] = 6;
                        ack_packet[4] = 1;
                        applyCheckSum(ack_packet, 6);
                        sendUART(6, ack_packet, 0);
                        shift_buffer(startIndex+1, stack, sizeof(stack));
                        stackSize -= startIndex+1;
                        break;
                    }
                case 3: // Call function
                    switch(stack[startIndex+3]){ //Instruction (ParameterN is at startIndex+3+N)
                        case 1: //Ping
                            sendUART(6, stack, startIndex);
                            break;
                        case 2: //Read(x, y)
                            readPosition();
//                            ack_packet = {255, 255, 6, 2, cur_pos_x/255, cur_pos_x%255, cur_pos_y/255, cur_pos_y%255, 0};
                            ack_packet[2] = 6;
                            ack_packet[3] = 2;
                            ack_packet[4] = 170; // X-high byte
                            ack_packet[5] = 187; // X-low byte
                            ack_packet[6] = 204; // Y-high byte
                            ack_packet[7] = 221; // Y-low byte
                            applyCheckSum(ack_packet, 9);
                            sendUART(9, ack_packet, 0);
                            break;
                        case 3: //Write (x, y)
                            writePosition(256*stack[startIndex+4] + stack[startIndex+5],256*stack[startIndex+6] + stack[startIndex+7]);
                            break;
                        case 4: //Write Trajectory
                            writeTrajectory(((stack[startIndex+4]==0)?1.0:-1.0)*(((float)stack[startIndex+5])+((float)stack[startIndex+6])/100+((float)stack[startIndex+7])/10000),
                                    ((stack[startIndex+8]==0)?1.0:-1.0)*(((float)stack[startIndex+9])+((float)stack[startIndex+10])/100+((float)stack[startIndex+11])/10000),
                                    cur_pos_x,
                                    cur_pos_y,
                                    ((stack[startIndex+12]==0)?1.0:-1.0)*(((float)stack[startIndex+13])+((float)stack[startIndex+14])/100+((float)stack[startIndex+15])/10000),
                                    ((stack[startIndex+16]==0)?1.0:-1.0)*(((float)stack[startIndex+17])+((float)stack[startIndex+18])/100+((float)stack[startIndex+19])/10000),
                                    (((float)stack[startIndex+20])+((float)stack[startIndex+21])/100+((float)stack[startIndex+22])/10000));
//                            ack_packet = {255, 255, 3, 4, 0, 0};
//                            ack_packet[2] = 3;
//                            ack_packet[3] = 4;
//                            ack_packet[4] = 0;
//                            applyCheckSum(ack_packet, 6);
//                            sendUART(6, ack_packet, 0);
                            break;
                        case 5: // Home
                            if(stack[startIndex+4]){ // Asked if home was set
//                                ack_packet = {255, 255, 3, 5, homed, 0};
//                                ack_packet[2] = 3;
//                                ack_packet[3] = 5;
//                                ack_packet[4] = homed;
//                                applyCheckSum(ack_packet, 6);
//                                sendUART(6, ack_packet, 0);
                            }
                            else{
                                setHome();
                            }
                            break;
                        case 7: // Motion 0=Circular
                            if(stack[startIndex+4] == 0){
                                circle_running = true;
                                circular_nf = stack[startIndex+5];
                                t=0;
                            }
                            break;
                        default:
                            Nop();
                    }
                    shift_buffer(startIndex+packetLength+3, stack, sizeof(stack));//Clear used stack
                    stackSize -= startIndex+packetLength+3;
                    uart_state = 0;
            }
        }
        if(circle_running){
            circularMotion(circular_nf);
        }
    }
    return 0;
}
void readPosition(){
    cur_pos_x = (int)POS1CNT*81.6/(3413.0*2.0);
    cur_pos_y = (int)POS2CNT*81.6/(3413.0*2.0);
}
void writePosition(float x, float y){

}
void sendAck(){
    ack_packet[2] = 3;
    ack_packet[3] = 5;
    ack_packet[4] = 1;
    applyCheckSum(ack_packet, 6);
    sendUART(6, ack_packet, 0);
}
void setHome(){
    while(t > 3)
    {
        Nop();
    }
//    ack_packet = {255, 255, 3, 5, 1, 0};
    ack_packet[2] = 3;
    ack_packet[3] = 5;
    ack_packet[4] = 1;
    applyCheckSum(ack_packet, 6);
    sendUART(6, ack_packet, 0);
}
void circularMotion(int nf){
    while(t > 3)
    {
        Nop();
    }
//    ack_packet = {255, 255, 3, 5, 1, 0};
    ack_packet[2] = 3;
    ack_packet[3] = 5;
    ack_packet[4] = 1;
    applyCheckSum(ack_packet, 6);
    sendUART(6, ack_packet, 0);
}
void recMotion(int nf){
    
}
void setup(){
     __builtin_disable_interrupts();
     initPLL();
    // // motor PWM setup

     T1CONbits.TCKPS = 0b01; //set timer prescaler to 1:64 interrupt
     T2CONbits.TCKPS = 0b10;
     T3CONbits.TCKPS = 0b01; // set timer PWM

     PR1 = 10000;
     PR2 = 12500;             //set period interrupt
     PR3 = 12500;            // set period PWM

     _T1IE = 1; 
     _T2IE = 1;   
            //enable interrupt for timer1
     _T2IP = 3;
     _T1IP = 2;

     OC1RS = 8000;// X 
     OC1CONbits.OCM = 0b000; 
     OC1CONbits.OCTSEL = 1; 
     OC1CONbits.OCM = 0b110; 
    // //port  AB

     OC2RS = 8000; // Y
     OC2CONbits.OCM = 0b000;
     OC2CONbits.OCTSEL = 1; 
     OC2CONbits.OCM = 0b110; 
    // //port  AB

     __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
     
    RPINR18bits.U1RXR = 6;  // RX pin setup
    RPOR2bits.RP5R = 3;     // TX pin setup

    RPINR14bits.QEA1R = 8;     //remap RP14 connect to QEI1_A
    RPINR14bits.QEB1R = 7;     //remap RP15 connect to QEI1_B
    
    RPINR16bits.QEA2R = 9;     //remap RP14 connect to QEI2_A
    RPINR16bits.QEB2R = 10;     //remap RP15 connect to QEI2_B

     _RP14R = 0b10010;
     _RP4R = 0b10011;

     __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK 
     UART1_Initialize(86, 347);

    /*QEI Mode Select*/
    QEI1CONbits.QEIM = 0b000;           // QEI Mode disable
    QEI1CONbits.PCDOUT = 0;             // no direction pin out
    QEI1CONbits.QEIM = 0b101;           // 4x ,no index
    /*digital filter config */
    DFLT1CONbits.QECK = 0b000;          // clock divider Fcy/1
    DFLT1CONbits.QEOUT = 1;             // enable filter

    QEI2CONbits.QEIM = 0b000;           // QEI Mode disable
    QEI2CONbits.PCDOUT = 0;             // no direction pin out
    QEI2CONbits.QEIM = 0b101;           // 4x ,no index
    /*digital filter config */
    DFLT2CONbits.QECK = 0b000;          // clock divider Fcy/1
    DFLT2CONbits.QEOUT = 1;             // enable filter
    
    /// DMA TX Setup ///
    IEC4bits.U1EIE = 0;
    DMA0REQ = 0x000C;					// Select UART1 Transmitter // Associate DMA Channel 0 with UART Tx
	DMA0PAD = (volatile unsigned int) &U1TXREG;
    DMA0CONbits.AMODE = 0;
	DMA0CONbits.MODE  = 1;
	DMA0CONbits.DIR   = 1;
	DMA0CONbits.SIZE  = 0;
	// DMA0CNT = 7;						// 8 DMA requests
    DMA0STA = __builtin_dmaoffset(BufferA);
    IFS0bits.DMA0IF  = 0;			// Clear DMA Interrupt Flag
	IEC0bits.DMA0IE  = 1;			// Enable DMA interrupt
    U1STAbits.UTXISEL0 = 0;			// Interrupt after one Tx character is transmitted (DMA need to interrupt UART)
	U1STAbits.UTXISEL1 = 0;
    U1MODEbits.UARTEN   = 1;		// Enable UART
	U1STAbits.UTXEN     = 1;		// Enable UART Tx
    IEC4bits.U1EIE = 0;
     __builtin_enable_interrupts();

    T3CONbits.TON =1;            //enable timer3
    AD1PCFGL = 0xFFFF;// set digital pin
    TRISB = 0x1FC1;
    TRISA = 0xFFFF;
}
float read_posX()
{   
    float pos_x = 150.25;
    return pos_x;
}
float read_posY()
{
    float pos_y = 149.52;
    return pos_y;
}

void delay()
{
    int i = 0;
    int j = 0;
    for(i=0;i<=1000;i++)
    {
        for(j = 0;j<= 4000;j++)
        {
            Nop();
        }
    }
    i = 0;
}
void applyCheckSum(uint8_t *buffer, int length){
    int i;
    uint8_t checksum = 0;
    for(i=2;i<length-1;i++){
        checksum += buffer[i];
    }
    buffer[length-1] = ~checksum;
}
void shift_buffer(int n, uint8_t *buffer, int buffer_size){ // Shift buffer to the left N times
    int i, j;
    for(i=0;i<n;i++){
        for(j=0; j<buffer_size-1; j++){
            buffer[j] = buffer[j+1];
        }
    }
}
void writeTrajectory(float C3, float C4, float X0, float Y0, float THETA, float GRAMMA, float TF)
{
    ti = 0;
    c1 = 0;
    c2 = 0;
    c3 = C3;
    c4 = C4;
    x0 = X0;
    y0 = Y0;
    theta = THETA;
    gramma = GRAMMA;
    path_mode = 0;
    tf = TF;
    t = 0;
}
void sendUART(int length, uint8_t *buffer, int startIndex)
{
    DMA0CNT = length - 1; // Length
    int i;
    BufferA[0] = 255; BufferA[1] = 255;
    for(i=2;i<length;i++){
        BufferA[i] = buffer[i + startIndex];
    }
    // Send !!!
    DMA0STA = __builtin_dmaoffset(BufferA);
    DMA0CONbits.CHEN  = 1;
    DMA0REQbits.FORCE = 1;
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
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
	IFS0bits.DMA0IF = 0;			// Clear the DMA0 Interrupt Flag;
}
void __attribute__((interrupt,no_auto_psv)) _T1Interrupt(void)
{  
    t = t+CON_T;
     _T1IF =0 ;  //clear interrupt flag
    
}
void __attribute__((interrupt,no_auto_psv)) _T2Interrupt(void)
{
    //numByte = UART1_ReadBuffer(dataArray, 10);
//    printf("w = :%f:%f:%f:%f:%f:%f:\n",t,(float)POS1CNT*81.6/(3413.0*2.0),(float)POS2CNT*81.6/(3413.0*2.0)  ,setpoint_y, vel_y ,x2_y);
     _T2IF =0 ;  //clear interrupt flag

}
