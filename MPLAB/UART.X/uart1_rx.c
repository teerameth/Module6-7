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
float prev_pos_x;
float prev_pos_y;
float pwm_x, kp_x = 1.0, ki_x = 0.0, kd_x = 2.0, prev_err_x, setpoint_x, i_term_x, d_term_x, feedback_x, new_error_x;
float pwm_vel_x, kp_vel_x = 1.2, ki_vel_x = 0.012, kd_vel_x = 2.5, prev_err_vel_x, setpoint_vel_x, i_term_vel_x, d_term_vel_x, feedback_vel_x, new_error_vel_x, vel_x=0;
float pwm_y, kp_y = 0.50, ki_y = 0.0, kd_y = 2.0, prev_err_y, setpoint_y, i_term_y, d_term_y, feedback_y, new_error_y;
float pwm_vel_y, kp_vel_y = 1.0, ki_vel_y = 0.01, kd_vel_y = 2.0, prev_err_vel_y, setpoint_vel_y, i_term_vel_y, d_term_vel_y, feedback_vel_y, new_error_vel_y, vel_y=0;
float sigma_a_x =8; // adjustable
float sigma_w_x =1.50; // adjustable

float w_update_x =0, w_inKalman_x = 0, w_outKalman_x = 0; 
// float kumara_base_yaw =0;
float Q_x,R_x, x1_in_x = 0, x2_in_x = 0;
float x1_x = 0; // orientation
float x2_x = 0; // w_outKalman (angular_vel)
float p11_x = 1.0; // adjustable
float p12_x = 0, p21_x = 0;
float p22_x = 0.9; // adjustable
float cur_theta_x, prev_theta_x;

float sigma_a_y =8; // adjustable
float sigma_w_y =1.50; // adjustable

float w_update_y =0;
float w_inKalman_y = 0;
float w_outKalman_y = 0; 
// float kumara_base_yaw =0;
float Q_y,R_y, x1_in_y = 0, x2_in_y = 0;
float x1_y = 0; // orientation
float x2_y = 0; // w_outKalman (angular_vel)
float p11_y = 1.0, p12_y = 0, p21_y = 0, p22_y = 0.9; // adjustable
float cur_theta_y, prev_theta_y;

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
void motorY(int speed);
void motorX(int speed);

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
//                    printf("packetLength = %d\n", packetLength);
//                    printf("stackSize = %d\n", stackSize);
//                    printf("startIndex = %d\n", startIndex);
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
//                        for (i=0;i<sizeof(stack);i++){printf("%d ", stack[i]);}printf("\n");
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
                            ack_packet[4] = cur_pos_x/256; // X-high byte
                            ack_packet[5] = cur_pos_x%256; // X-low byte
                            ack_packet[6] = cur_pos_y/256; // Y-high byte
                            ack_packet[7] = cur_pos_y%256; // Y-low byte
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
                            }
                            break;
                        default:
                            Nop();
                    }
//                    printf("stackSize = %d\n", stackSize);
//                    for(i=0;i<sizeof(stack);i++){printf("%d ", stack[i]);}printf("\n");
                    shift_buffer(startIndex+packetLength+3, stack, sizeof(stack));//Clear used stack
                    stackSize -= startIndex+packetLength+3;
//                    for(i=0;i<sizeof(stack);i++){printf("%d ", stack[i]);}printf("\n");
//                    printf("stackSize = %d\n", stackSize);
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
    setpoint_x = x;
    setpoint_y = y;
    setpoint_vel_x = 0;
    setpoint_vel_y = 0;
}
void setHome(){
    while(_RB12)
    {
        motorX(-20);//-19
    }
    motorX(0);
    
    while(_RB11)
    {
        motorY(-20);//1
    }
    motorY(0);
    delay();
    POS2CNT = 0;
    POS1CNT = 0; 
    T1CONbits.TON =1;  
    T2CONbits.TON =1;
//    ack_packet = {255, 255, 3, 5, 1, 0};
    ack_packet[2] = 3;
    ack_packet[3] = 5;
    ack_packet[4] = 1;
    applyCheckSum(ack_packet, 6);
    sendUART(6, ack_packet, 0);
    homed = true;
}
void circularMotion(int nf){
    float tf1 = 6.28;
    if(t<3)
    {
        ti = 0;
        c1 = 0;
        c2 = 0;
        c3 = 83.396;
        c4 = -18.53;
        x0 = 0;
        y0 = 0;
        theta = 1.53;
        gramma = 0;
        path_mode = 0;
        tf = 3;  
    }  
    else if(t<8 && t>3)
    {
        c1 = 0;
        c2 = 0;
        c3 = -0.17699;
        c4 = 0.0102;
        r = 130;
        phi = PI;
        gramma = 0;
        ti = 3;
        x0 = 140;
        y0 = 250;
        path_mode = 1;
        tf = 8;  
    } 
    else if((t < (tf1*n)+8) && n <= nf && t > 8)
    {
        ti = (tf1*(n-1))+8;  
        c1 = 0;
        c2 = -1;
        c3 = 0;
        c4 = 0;
        phi = 0;
        r = 130;
        x0 = 140;
        y0 = 250;
        path_mode = 1;
        gramma = 0;
        tf = (tf1*n)+8; 
    }
    
    else if((t < (tf1*n)+8) && n > nf && t > 8)
    {
        ti = (tf1*(n-1))+8; 
        c1 = 0;
        c2 = -1;
        c3 = 0.023;
        c4 = 0.0102;
        r = 130;
        phi = 0;
        gramma = 0;
        x0 = 140;
        y0 = 250;
        path_mode = 1;
        tf = (tf1*n)+8;
    }
    if((t > (tf1*n)+8) && n <= nf)
    {
        n++;
    }
}
void recMotion(int nf){
    if(t<4)
    {
        ti = 0;
        c1 = 0;
        c2 = 0;
        c3 = 22.6779;
        c4 = -3.7629;
        x0 = 0;
        y0 = 0;
        theta = 1.487;
        gramma = 0;
        path_mode = 0;
        tf = 4;  
    }
    else if(t<8 && t > 4)
    {
        ti = 4;
        c1 = 0;
        c2 = 0;
        c3 = 48.75;
        c4 = -8.125;
        x0 = 10;
        y0 = 120;
        theta = 0;
        gramma = 0;
        path_mode = 0;
        tf = 8;  
    }
    else if(t<12 && t > 8)
    {
        ti = 8;
        c1 = 0;
        c2 = 0;
        c3 = 48.75;
        c4 = -8.125;
        x0 = 270;
        y0 = 120;
        theta = PI/2;
        gramma = 0;
        path_mode = 0;
        tf = 12;  
    }
    else if(t<16 && t > 12)
    {
        ti = 12;
        c1 = 0;
        c2 = 0;
        c3 = 48.75;
        c4 = -8.125;
        x0 = 270;
        y0 = 380;
        theta = PI;
        gramma = 0;
        path_mode = 0;
        tf = 16;  
    }
    else if(t<20 && t > 16)
    {
        ti = 16;
        c1 = 0;
        c2 = 0;
        c3 = 48.75;
        c4 = -8.125;
        x0 = 10;
        y0 = 380;
        theta = -PI/2;
        gramma = 0;
        path_mode = 0;
        tf = 20;  
    }
    if(t>20 && n < nf)
    {
        n++;
        t = 4;
        ti = 4;
        c1 = 0;
        c2 = 0;
        c3 = 48.75;
        c4 = -8.125;
        x0 = 10;
        y0 = 120;
        theta = 0;
        gramma = 0;
        path_mode = 0;
        tf = 8; 
    }
    if(t>20 && n <= nf)
    {
        n++;
    }
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
    float pos_x = (float)POS1CNT*81.6/(3413.0*2.0);
    if(pos_x - prev_pos_x > 200)
    {
        pos_x = 0;
    }
    prev_pos_x = pos_x;
    return pos_x;
}
float read_posY()
{
    float pos_y = (float)POS2CNT*81.6/(3413.0*2.0);
    if(pos_y - prev_pos_y > 200)
    {
        pos_y = 0;
    }
    prev_pos_y = pos_y;
    return pos_y;
}
void motorY(int speed)
{
    if (speed>100){
        speed = 100;
    }
    if (speed < -100){
        speed = -100;
    }
    if(speed > 0){
        _LATB15 = 1; //A1
        _LATB13 = 0; //B1
    }
    else if(speed < 0){
        _LATB15 = 0; //A1
        _LATB13 = 1; //B1
    }
    else{
        _LATB15 = 0; //A1
        _LATB13 = 0; //B1        
    }
    speed = abs(speed);
    unsigned long pwm = speed*((unsigned long)PR3);
    pwm /= 100;
    OC1RS = pwm;
}
void motorX(int speed)
{
    if (speed>100)
    {
        speed = 100;
    }
    if (speed < -100)
    {
        speed = -100;
    }
    if(speed > 0){
        _LATB2 = 0; //A1
        _LATB3 = 1; //B1
    }
    else if(speed < 0)
    {
        _LATB2 = 1; //A1
        _LATB3 = 0; //
    }
    else
    {
        _LATB2 = 0; //A1
        _LATB3 = 0; //B1
    }

    speed = abs(speed);
    unsigned long pwm = speed*((unsigned long)PR3);
    pwm /= 100;
    OC2RS = pwm;
    
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
    buffer[sizeof(buffer)-1] = ~checksum;
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
    for(i=0;i<length;i++){
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
    if(t < tf)
    {
        Q_t =c1 + c2 *(t-ti) + c3*(t-ti)*(t-ti) + c4*(t-ti)*(t-ti)*(t-ti);
        Q_dot_t = c2 + 2*c3*(t-ti) + 3*c4*(t-ti)*(t-ti);

        if(path_mode == 0)
        {
            setpoint_x = x0 + (Q_t)*cos(gramma)*cos(theta);
            vel_x = (Q_dot_t)*cos(gramma)*cos(theta);
            setpoint_y = y0 + (Q_t)*cos(gramma)*sin(theta);
            vel_y = (Q_dot_t)*cos(gramma)*sin(theta);
        }
    
        else if(path_mode == 1)
        { 
            setpoint_x = (x0+r*cos(Q_t+phi));
            setpoint_y = (y0+r*sin(Q_t+phi));
            vel_x = (-r*sin(Q_t+phi)*Q_dot_t);
            vel_y = r*cos(Q_t+phi)*Q_dot_t;
        }
        trajectory_running = true;
    }
    else{
        if(trajectory_running == true){
//            ack_packet = {255, 255, 3, 4, 1, 0};
//            ack_packet[2] = 3;
//            ack_packet[3] = 4;
//            ack_packet[4] = 1;
//            applyCheckSum(ack_packet, 6);
//            sendUART(6, ack_packet, 0);
            trajectory_running = false;
        }
    }
    //---------------------- x axis control --------------------------

    cur_theta_x = read_posX();
    w_inKalman_x = (cur_theta_x-prev_theta_x)/CON_T;    
    
    Q_x = sigma_a_x*sigma_a_x;
    R_x = sigma_w_x*sigma_w_x;

    ///////////
    float x1_new_x = x1_in_x+x2_in_x*CON_T;
    float x2_new_x = 0+x2_in_x;
    float ye_x = w_inKalman_x-x2_new_x;

    //สมการหลัก///
    p11_x = p11_x + CON_T*p21_x + (Q_x*CON_T*CON_T*CON_T*CON_T)/4 + ((CON_T*CON_T)*(p12_x + CON_T*p22_x))/CON_T;
    p12_x = p12_x + CON_T*p22_x + (Q_x*CON_T*CON_T*CON_T)/2;
    p21_x = (2*CON_T*p21_x + Q_x*CON_T*CON_T*CON_T*CON_T + 2*p22_x*CON_T*CON_T)/(2*CON_T);
    p22_x = Q_x*CON_T*CON_T + p22_x;
    x1_new_x = x1_new_x + (p12_x*ye_x)/(R_x + p22_x);
    x2_new_x = x2_new_x + (p22_x*ye_x)/(R_x + p22_x);
    p11_x = p11_x - (p12_x*p21_x)/(R_x + p22_x);
    p12_x = p12_x - (p12_x*p22_x)/(R_x + p22_x);
    p21_x = -p21_x*(p22_x/(R_x + p22_x) - 1);
    p22_x = -p22_x*(p22_x/(R_x + p22_x) - 1);
    x1_x = x1_new_x;
    x2_x = x2_new_x;


    feedback_x = read_posX();
    new_error_x = setpoint_x - feedback_x;
    i_term_x += new_error_x;
    d_term_x = new_error_x-prev_err_x;
    pwm_x = kp_x*new_error_x + ki_x*i_term_x+kd_x*d_term_x;
   
    setpoint_vel_x =  pwm_x +  vel_x;

    // vel control
    feedback_vel_x = x2_x;
    new_error_vel_x = setpoint_vel_x - feedback_vel_x;
    i_term_vel_x += new_error_vel_x;
    d_term_vel_x = new_error_vel_x-prev_err_vel_x;
    pwm_vel_x = kp_vel_x*new_error_vel_x + ki_vel_x*i_term_vel_x+kd_vel_x*d_term_vel_x;
    
    motorX(pwm_vel_x);

    prev_err_vel_x = new_error_vel_x;
    prev_err_x = new_error_x;
    prev_theta_x = cur_theta_x;
    x1_in_x = x1_x;
    x2_in_x = x2_x;

    //---------------------------- y axis control -------------------------
    
    cur_theta_y = read_posY();
    w_inKalman_y = (cur_theta_y-prev_theta_y)/CON_T;    
    
    Q_y = sigma_a_y*sigma_a_y;
    R_y = sigma_w_y*sigma_w_y;

    ///////////
    float x1_new_y = x1_in_y + x2_in_y *CON_T;
    float x2_new_y = 0+x2_in_y;
    float ye_y = w_inKalman_y-x2_new_y;

    //สมการหลัก///
    p11_y = p11_y + CON_T*p21_y + (Q_y*CON_T*CON_T*CON_T*CON_T)/4 + ((CON_T*CON_T)*(p12_y + CON_T*p22_y))/CON_T;
    p12_y = p12_y + CON_T*p22_y + (Q_y*CON_T*CON_T*CON_T)/2;
    p21_y = (2*CON_T*p21_y + Q_y*CON_T*CON_T*CON_T*CON_T + 2*p22_y*CON_T*CON_T)/(2*CON_T);
    p22_y = Q_y*CON_T*CON_T + p22_y;
    x1_new_y = x1_new_y + (p12_y*ye_y)/(R_y + p22_y);
    x2_new_y = x2_new_y + (p22_y*ye_y)/(R_y + p22_y);
    p11_y = p11_y - (p12_y*p21_y)/(R_y + p22_y);
    p12_y = p12_y - (p12_y*p22_y)/(R_y + p22_y);
    p21_y = -p21_y*(p22_y/(R_y + p22_y) - 1);
    p22_y = -p22_y*(p22_y/(R_y + p22_y) - 1);
    x1_y = x1_new_y;
    x2_y = x2_new_y;

    // pos control
    feedback_y = read_posY();
    new_error_y = setpoint_y - feedback_y;
    i_term_y += new_error_y;
    d_term_y = new_error_y-prev_err_y;
    pwm_y = kp_y*new_error_y + ki_y*i_term_y + kd_y*d_term_y;
   
    setpoint_vel_y = pwm_y + vel_y;

    // vel control
    feedback_vel_y = x2_y;
    new_error_vel_y = setpoint_vel_y - feedback_vel_y;
    i_term_vel_y += new_error_vel_y;
    d_term_vel_y = new_error_vel_y-prev_err_vel_y;
    pwm_vel_y = kp_vel_y*new_error_vel_y + ki_vel_y*i_term_vel_y+kd_vel_y*d_term_vel_y;
    
    motorY(pwm_vel_y);

    prev_err_vel_y = new_error_vel_y;
    prev_err_y = new_error_y;
    prev_theta_y = cur_theta_y;
    x1_in_y = x1_y;
    x2_in_y = x2_y;

    t = t+CON_T;
     _T1IF =0 ;  //clear interrupt flag
    
}
void __attribute__((interrupt,no_auto_psv)) _T2Interrupt(void)
{
    //numByte = UART1_ReadBuffer(dataArray, 10);
//    printf("w = :%f:%f:%f:%f:%f:%f:\n",t,(float)POS1CNT*81.6/(3413.0*2.0),(float)POS2CNT*81.6/(3413.0*2.0)  ,setpoint_y, vel_y ,x2_y);
     _T2IF =0 ;  //clear interrupt flag

}
