/*
 * File:   uart1.c
 * Author: G_Peav
 *
 * Created on September 9, 2019, 4:31 PM
 */


#include "xc.h"
#include "stdio.h"
#include "configuration.h"
#include "UART_rev3.h"
#include <math.h>

#define PI 3.14159265
#define G 9.782970341
#define CON_T 0.010

float pwm_x;
float kp_x = 1.0;
float ki_x = 0.0;
float kd_x = 7.0;;
float prev_err_x;
float setpoint_x;
float i_term_x;
float d_term_x;
float feedback_x;
float new_error_x;

float pwm_vel_x;
float kp_vel_x = 0.970;
float ki_vel_x = 0.021;
float kd_vel_x = 0.25;
float prev_err_vel_x;
float setpoint_vel_x;
float i_term_vel_x;
float d_term_vel_x;
float feedback_vel_x;
float new_error_vel_x;
float vel_x=0;

float pwm_y;
float kp_y = 1.0;
float ki_y = 0.0;
float kd_y = 7.0;
float prev_err_y;
float setpoint_y;
float i_term_y;
float d_term_y;
float feedback_y;
float new_error_y;

float pwm_vel_y;
float kp_vel_y = 1.60;
float ki_vel_y = 0.045;
float kd_vel_y = 0.3;
float prev_err_vel_y;
float setpoint_vel_y;
float i_term_vel_y;
float d_term_vel_y;
float feedback_vel_y;
float new_error_vel_y;
float vel_y=0;

float sigma_a_x =14; // adjustable
float sigma_w_x =1.30; // adjustable

float w_update_x =0;
float w_inKalman_x = 0;
float w_outKalman_x = 0; 
// float kumara_base_yaw =0;
float Q_x,R_x; 
float x1_in_x = 0;
float x2_in_x = 0;
float x1_x = 0; // orientation
float x2_x = 0; // w_outKalman (angular_vel)
float p11_x = 1.0; // adjustable
float p12_x = 0;
float p21_x = 0;
float p22_x = 0.9; // adjustable
float cur_theta_x;
float prev_theta_x;

float sigma_a_y =14; // adjustable
float sigma_w_y =1.30; // adjustable

float w_update_y =0;
float w_inKalman_y = 0;
float w_outKalman_y = 0; 
// float kumara_base_yaw =0;
float Q_y,R_y; 
float x1_in_y = 0;
float x2_in_y = 0;
float x1_y = 0; // orientation
float x2_y = 0; // w_outKalman (angular_vel)
float p11_y = 1.0; // adjustable
float p12_y = 0;
float p21_y = 0;
float p22_y = 0.9; // adjustable
float cur_theta_y;
float prev_theta_y;

float t,tf,ti;
float r,theta;
float c1,c2,c3,c4;
float theta;
int state = 0;
float x0,y0;
float Q_t;
float Q_dot_t;
int path_mode;
float phi;

int n  = 1;

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
    float cur_pos_x = (float)POS1CNT*81.6/(3413.0*2.0);
    float cir_pos_y = (float)POS2CNT*81.6/(3413.0*2.0);
}
void writePosition(float x, float y){
    // Acknowledge
    // Move
    // Report
}
void writeTrajectory(float Tf, float C1, float C2, float C3, float C4, float X0, float Y0, float Theta){
    tf = Tf;
    c1 = C1;
    c2 = C2;
    c3 = C3;
    c4 = C4;
    x0 = X0;
    y0 = Y0;
    theta = Theta;
    path_mode = 0;
    ti = 0;
    // Acknowledge
    // Move
    // Report


    //        ti = 0;
//        c1 = 0;
//        c2 = 0;
//        c3 = 43.266;
//        c4 = -5.7688;
//        x0 = 0;
//        y0 = 0;
//        theta = 0.588;
}
void setHome(){
    while(!_RA4)
    {
        motorX(-30);//-19
    }
    motorX(0);
    
    while(!_RB12)
    {
        motorY(-25);//1
    }
    motorY(0);
    delay();
    POS2CNT = 0;
    POS1CNT = 0; 
    T1CONbits.TON =1;  
    T2CONbits.TON =1;
    // Acknowledge
    // Move
    // Report
}
void askHome(){
    // Acknowledge
    // Move
    // Report
}
void circularMotion(int nf){
    float tf1 = 15;
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
        path_mode = 0;
        tf = 3;  
    }   
    else if(t < (tf1*n)+3)
    {
        ti = (tf1*(n-1))+3;  
        c1 = 0;
        c2 = 0;
        c3 = -0.0837;
        c4 = 0.00372;
        phi = PI;
        r = 130;
        x0 = 140;
        y0 = 250;
        path_mode = 1;
        tf = (tf1*n)+3; 
    }
    if((t > (tf1*n)+3) && n < nf)
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
     T3CONbits.TCKPS = 0b00; // set timer PWM

     PR1 = 50000;
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
     
    RPINR18bits.U1RXR = 6;
    RPOR2bits.RP5R = 0b00011;

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

     __builtin_enable_interrupts();
 
    T3CONbits.TON =1;            //enable timer3
    AD1PCFGL = 0xFFFF;// set digital pin
    TRISB = 0x1FC1;
    TRISA = 0xFFFF;
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

void __attribute__((interrupt,no_auto_psv)) _T1Interrupt(void)
{  
    //-----------------------trajectory
    if(t < tf)
    {
        Q_t =c1 + c2 *(t-ti) + c3*(t-ti)*(t-ti) + c4*(t-ti)*(t-ti)*(t-ti);
        Q_dot_t = c2 + 2*c3*(t-ti) + 3*c4*(t-ti)*(t-ti);

        if(path_mode == 0)
        {
            setpoint_x = x0 + (Q_t)*cos(theta);
            vel_x = (Q_dot_t)*cos(theta);
            setpoint_y = y0 + (Q_t)*sin(theta);
            vel_y = (Q_dot_t)*sin(theta);
        }
    
        else if(path_mode == 1)
        { 
            setpoint_x = (x0+r*cos(Q_t+phi));
            setpoint_y = (y0+r*sin(Q_t+phi));
            vel_x = (-r*sin(Q_t)*Q_dot_t);
            vel_y = r*cos(Q_t)*Q_dot_t;
        }
    }
    //---------------------- x axis control --------------------------

    cur_theta_x = (float)POS1CNT*81.6/(3413.0*2.0);
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


    feedback_x = (float)POS1CNT*81.6/(3413.0*2.0);
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
    
    cur_theta_y = (float)POS2CNT*81.6/(3413.0*2.0);
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
    feedback_y = (float)POS2CNT*81.6/(3413.0*2.0);
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
    printf("w = :%f:%f:%f:%f:%f:%f:\n",t,(float)POS1CNT*81.6/(3413.0*2.0),(float)POS2CNT*81.6/(3413.0*2.0)  ,setpoint_y, vel_y ,x2_y);
     _T2IF =0 ;  //clear interrupt flag

}
int main(void) {
    setup();
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
                            readPosition();
                            printf("Instruction 2\n");
                            break;
                        case 3: //Write (x, y)
                            printf("Instruction 3\n");
                            break;
                        case 4: //Write Trajectory
                            printf("Instruction 4\n");
                            break;
                        case 5: // Home
                            setHome();
                            printf("Instruction 5\n");
                            break;
                        case 7: // Motion
                            circularMotion(2)
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
