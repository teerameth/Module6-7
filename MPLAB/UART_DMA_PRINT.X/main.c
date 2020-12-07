#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include "configuration.h"
#include "UART_rev3.h"
#include <stdbool.h> 
#include <math.h>
#include <string.h>

#define PI 3.14159265
#define G 9.782970341
#define CON_T 0.002
float prev_pos_x;
float prev_pos_y;
float pwm_x, kp_x = 1.8, ki_x = 0.0, kd_x = 3.0, prev_err_x, setpoint_x, i_term_x, d_term_x, feedback_x, new_error_x;
float pwm_vel_x, kp_vel_x =1.5, ki_vel_x = 0.035, kd_vel_x = 5.5, prev_err_vel_x, setpoint_vel_x, i_term_vel_x, d_term_vel_x, feedback_vel_x, new_error_vel_x, vel_x=0;
float pwm_y, kp_y = 1.8, ki_y = 0.0, kd_y = 3.0, prev_err_y, setpoint_y, i_term_y, d_term_y, feedback_y, new_error_y;
float pwm_vel_y, kp_vel_y = 1.50, ki_vel_y = 0.035, kd_vel_y = 5.5, prev_err_vel_y, setpoint_vel_y, i_term_vel_y, d_term_vel_y, feedback_vel_y, new_error_vel_y, vel_y=0;
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
uint8_t stackSize=0, startIndex, packetLength, uart_state = 0, checksum, circular_nf;
bool circle_running = false, trajectory_running = false, homed = false, circle_start = false;
int cur_pos_x, cur_pos_y;
uint8_t ack_packet[30] = {255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int i;
void initPLL();
void setup();
float read_posX();
float read_posY();
void readPosition();
void writePosition(float x, float y);
void writeTrajectory(float C3, float C4, float X0, float Y0, float THETA, float GRAMMA, float TF);
void applyCheckSum(uint8_t *buffer, int length);
void shift_buffer(int n, uint8_t *buffer, int buffer_size);
void setHome();
void circularMotion(int nf);
void recMotion(int nf);
void delay();
void motorY(int speed);
void motorX(int speed);
void sendAck();
void send();
void commie();
unsigned char BufferWrite[30] __attribute__((space(dma)));
unsigned char BufferRead[30] __attribute__((space(dma)));

int main(void) {
    /*disable global interrupt*/
    __builtin_disable_interrupts();
    initall();
    cfgDma1UartRx();
    cfgDma0UartTx();
    /*enable global interrupt*/
    __builtin_enable_interrupts();
    POS1CNT=0;
    POS2CNT=0;
    T4CONbits.TON = 1;
    T5CONbits.TON = 0;

    while(1){
        if(circle_running){circularMotion(circular_nf);}
    }
    return 0;  
} 


void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void){
	_DMA0IF = 0;			// Clear the DMA0 Interrupt Flag;
}
void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void){   
//    sprintf(BufferWrite,"a");
//    send();
    commie();
	_DMA1IF = 0;			// Clear the DMA1 Interrupt Flag
}


void initPLL(){
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
void initall(){
    initPLL();
    AD1PCFGL = 0xFFFF;      //set analog input to digital pin
    TRISB = 0x1FC1;
    TRISA = 0xFFFF;
//    _RB13 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    _RP14R = 0b10010;
     _RP4R = 0b10011;                //remap RP9 connect to OC2
    RPINR18bits.U1RXR = 6;           //remap RP6 connect to U1RX
    _RP5R = 0b00011;                 //remap RP5 connect to U1TX
//    RPINR19bits.U2RXR = 7;           //remap RP7 connect to U2RX
//    _RP3R = 0b00101;                 //remap RP3 connect to U2TX
    _QEA1R = 8;           //remap RP8 connect to QEI1_A
    _QEB1R = 7;           //remap RP9 connect to QEI1_B
    _QEA2R = 9;           //remap RP10 connect to QEI2_A
    _QEB2R = 10;           //remap RP11 connect to QEI2_B
    RPINR0bits.INT1R = 12;            //remap RP12 connect to INT1
    RPINR1bits.INT2R = 4;            //remap RP4 connect to INT2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
    /*uart1*/
    U1MODEbits.STSEL = 0;   // 1 Stop bit
    U1MODEbits.PDSEL = 0;   // No Parity, 8 data bits
    U1MODEbits.BRGH = 1;    // High Speed mode
    U1MODEbits.URXINV = 0;  // UxRX idle state is '1'
    U1BRG = 86;             // BAUD Rate Setting for 115200
    U1STAbits.UTXISEL0 = 0; // Interrupt after one TX Character is transmitted
    U1STAbits.UTXISEL1 = 0;		                            
	U1STAbits.URXISEL  = 0;			// Interrupt after one RX character is received
    U1MODEbits.UARTEN = 1;  // Enable UART
    U1STAbits.UTXEN = 1;    // Enable UART TX
    
    unsigned int i;             /* wait at least 347 usec (1/115200) before sending first char */
    for(i = 0; i < 347; i++){
        Nop();
    }
    /*timerInterrupt*/
    T1CONbits.TCKPS = 0b01;
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
     
    T4CONbits.TCKPS = 0b10;
    PR4 = 1250;
    _T4IE = 1;
    _T4IP = 7;
    T5CONbits.TCKPS = 0b10;
    PR5 = 12500;
    _T5IE = 1;
    _T5IP = 1;
    
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
     
    /*set INT*/
    IEC1bits.INT1IE = 0;
    IEC1bits.INT2IE = 0;
    INTCON2bits.INT1EP = 0;
    INTCON2bits.INT2EP = 0;
    IPC5bits.INT1IP = 6;
    IPC7bits.INT2IP = 6;
    /*set pwm*/
    
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
    
    PR3 = 12500;            // set period PWM
    
    T3CONbits.TON =1;            //enable timer3
}

void cfgDma0UartTx(void){
	//********************************************************************************
	//  STEP 3:
	//  Associate DMA Channel 0 with UART Tx
	//********************************************************************************/
	DMA0REQ = 0x000C;					// Select UART2 Transmitter
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
	// One-Shot, Post-Increment, RAM-to-Peripheral
	DMA0CONbits.AMODE = 0;
	DMA0CONbits.MODE  = 1;
	DMA0CONbits.DIR   = 1;
	DMA0CONbits.SIZE  = 1;
	DMA0CNT = 29;						// 30 DMA requests

	//********************************************************************************
	//  STEP 6:
	// Associate one buffer with Channel 0 for one-shot operation
	//********************************************************************************/
	DMA0STA = __builtin_dmaoffset(BufferWrite);

	//********************************************************************************
	//  STEP 8:
	//	Enable DMA Interrupts
	//********************************************************************************/
	IFS0bits.DMA0IF  = 0;			// Clear DMA Interrupt Flag
	IEC0bits.DMA0IE  = 1;			// Enable DMA interrupt

}
void cfgDma1UartRx(void){
	//********************************************************************************
	//  STEP 3:
	//  Associate DMA Channel 1 with UART Rx
	//********************************************************************************/
	DMA1REQ = 0x000B;					// Select UART1 Receiver
	DMA1PAD = (volatile unsigned int) &U1RXREG;

	//********************************************************************************
	//  STEP 4:
	//  Configure DMA Channel 1 to:
	//  Transfer data from UART to RAM Continuously
	//  Register Indirect with Post-Increment
	//  Using two ?ping-pong? buffers
	//  8 transfers per buffer
	//  Transfer words
	//********************************************************************************/
	//DMA1CON = 0x0002;					// Continuous,  Post-Inc, Periph-RAM
	DMA1CONbits.AMODE = 0;
	DMA1CONbits.MODE  = 0;
	DMA1CONbits.DIR   = 0;
	DMA1CONbits.SIZE  = 1;
	DMA1CNT = 29;						// 30 DMA requests

	//********************************************************************************
	//  STEP 6:
	//  Associate two buffers with Channel 1 for ?Ping-Pong? operation
	//********************************************************************************/
	DMA1STA = __builtin_dmaoffset(BufferRead);

	//********************************************************************************
	//  STEP 8:
	//	Enable DMA Interrupts
	//********************************************************************************/
	_DMA1IF  = 0;			// Clear DMA interrupt
	_DMA1IE  = 1;			// Enable DMA interrupt

	//********************************************************************************
	//  STEP 9:
	//  Enable DMA Channel 1 to receive UART data
	//********************************************************************************/
	DMA1CONbits.CHEN = 1;			// Enable DMA Channel
}

void send(){
    DMA0STA = __builtin_dmaoffset(BufferWrite);
    DMA0CNT = strlen(BufferWrite)-1;
    DMA0CONbits.CHEN  = 1;			// Re-enable DMA0 Channel
    DMA0REQbits.FORCE = 1;			// Manual mode: Kick-start the first transfer
}
void commie(){
    switch(BufferRead[2]){
        case 1: // Ping
            sprintf(BufferWrite, BufferRead);
            send();
            break;
        case 2: // Read (x, y)
            readPosition();
            sprintf(BufferWrite, "6,2,%d,%d,%d,%d", (int)cur_pos_x/256, (int)cur_pos_x%256, (int)cur_pos_y/256, (int)cur_pos_y%256);
            send();
            break;
        case 3:break; //writePosition
        case 4: //Write Trajectory
            writeTrajectory(((BufferRead[3]==0)?1.0:-1.0)*(((float)BufferRead[4])+((float)BufferRead[5])/100+((float)BufferRead[6])/10000),
                                    ((BufferRead[7]==0)?1.0:-1.0)*(((float)BufferRead[8])+((float)BufferRead[9])/100+((float)BufferRead[10])/10000),
                                    cur_pos_x,
                                    cur_pos_y,
                                    ((BufferRead[11]==0)?1.0:-1.0)*(((float)BufferRead[12])+((float)BufferRead[13])/100+((float)BufferRead[14])/10000),
                                    ((BufferRead[15]==0)?1.0:-1.0)*(((float)BufferRead[16])+((float)BufferRead[17])/100+((float)BufferRead[18])/10000),
                                    (((float)BufferRead[19])+((float)BufferRead[20])/100+((float)BufferRead[21])/10000));
//            sprintf(BufferWrite, "Start TJ");
//            send();
            break;
        case 5: // Home
            setHome();
           break;
        case 7: // Motion 0=Circular
            if(BufferRead[3] == 0){
                circle_running = true;
                circular_nf = BufferRead[4];
                t=0;
            }
            break;            
    }   
}
void readPosition(){
    cur_pos_x = (int)POS1CNT*81.6/(3413.0*2.0);
    cur_pos_y = (int)POS2CNT*81.6/(3413.0*2.0);
}

void circularMotion(int nf){
    float tf1 = 6.28;
    if(t<3)
    {
        ti = 0;
        c1 = 0;
        c2 = 0;
        c3 = 0;
        c4 = 0;
        x0 = 0;
        y0 = 0;
        theta = 0;
        gramma = 0;
        path_mode = 0;
        tf = 3;
        circle_start = true;
        t = 3;
    }
    if(t<8 && t>3)
    { // Start real circle
        if(circle_start){
            circle_start = false;
            sprintf(BufferWrite, "3,7,0");
            send();
        }
        c1 = 0;
        c2 = 0;
        c3 = -0.17699;
        c4 = 0.0102;
        r = 130;
        phi = PI;
        gramma = 0;
        ti = 3;
        x0 = 140;
        y0 = 150;
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
        y0 = 150;
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
        x0 = 140;
        y0 = 150;
        path_mode = 1;
        tf = (tf1*n)+6;
    }
    else if(t > (tf1*n)+8 && n > nf && t > 8){
        c1 = 0;
        c2 = 0;
        c3 = 0.0;
        c4 = 0.0;
        circle_running = false;
        sprintf(BufferWrite, "3,7,1");
        send();
        n = 1;
    }
    if((t > (tf1*n)+8) && n <= nf)
    {
        n++;
    }
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
void __attribute__((interrupt, no auto_psv)) _INT1Interrupt(void){
    IFS1bits.INT1IF = 0;  //clear interrupt flag 
}
void __attribute__((interrupt, no auto_psv)) _INT2Interrupt(void){
    IFS1bits.INT2IF = 0;  //clear interrupt flag 
}
void __attribute__((interrupt,no_auto_psv)) _T1Interrupt(void){
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
            sprintf(BufferWrite, "3,4,1");
            send();
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

void __attribute__((interrupt,no_auto_psv)) _T4Interrupt(void){
    
    _T4IF = 0;
}
void __attribute__((interrupt,no_auto_psv)) _T5Interrupt(void){
    _T5IF = 0;
}
void setHome(){
    T1CONbits.TON =0;  
    T2CONbits.TON =0;
    setpoint_x = 0;
    setpoint_y = 0;
    setpoint_vel_x = 0;
    setpoint_vel_y = 0;
    c1 = 0;
    c2 = 0;
    c3 = 0;
    c4 = 0;
    t = 0;
    tf = 0;
    i_term_x = 0;
    d_term_x = 0;
    i_term_vel_x = 0;
    d_term_vel_x = 0;
    i_term_y = 0;
    d_term_y = 0;
    i_term_vel_y = 0;
    d_term_vel_y = 0;
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
    sprintf(BufferWrite, "home");
    send();
    homed = true;
}