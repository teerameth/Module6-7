#include "BluetoothSerial.h"
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>
#define STEP_PER_ROUND 400
#define B_zero (int)(STEP_PER_ROUND/2)
#define motorInterfaceType 1
BluetoothSerial SerialBT;
hw_timer_t * timer = NULL;
// Note: 200cycles : 5mm.
const int dirPinA = 26; // 26 for old board
const int stepPinA = 25; // 25 for old board
const int dirPinB = 32;
const int stepPinB = 33;
//const int proximityPin = 27; // Active LOW
const int gripperServoPin = 14;
const int limitSwitchPin = 12; // Homing switch
float pulseDelay; // 1000 for 28byj-48, 500 for NEMA-17
bool led_state = false;
int gripper_pos = 0;
int stepAPos = 0, stepBPos = B_zero, stepADes = 0, stepBDes = 0;
Servo gripper_servo;
AccelStepper Zaxis(1, stepPinA, dirPinA); // pin 5 = step, pin 8 = direction

volatile long unsigned int t_stepA, t_stepB, t_stepACnt, t_stepBCnt, tA_left, tB_left;
double c1 = 0,c2 = 0,c3,c4,gramma;
volatile double theta_t, theta_dot_t, setpoint_z, vel_z, t,tf;
int reportCnt = 0;
int stepPin, dirPin, delta;
int deltaA, deltaB;
uint8_t stack[40], ack_packet[20];
uint8_t stackSize=0, startIndex, packetLength, uart_state = 0, checksum;
char byte_in;

void setZero();
void shift_buffer(int n, uint8_t *buffer, int buffer_size);
void applyCheckSum(uint8_t *buffer, int length);

void IRAM_ATTR onStepper(){
  
  reportCnt++;
  if(tB_left > 1)tB_left--;
  if(t<tf){
    theta_t = (((c4*t)+c3)*t+c2)*t+c1;
    theta_dot_t = ((3*c4*t)+c3*2)*t + c2;
    setpoint_z = (theta_t)*sin(gramma);
    vel_z = (theta_dot_t)*sin(gramma);
    //t_stepACnt++;
  }
//  else
//  {
//    Zaxis.setSpeed(0);
//  }
  deltaB = stepBDes - stepBPos;
  if(deltaB){
    if(deltaB > 0){
      digitalWrite(dirPinB, LOW);
      stepBPos++;
    }
    else{
      digitalWrite(dirPinB, HIGH);
      stepBPos--;
    }
    digitalWrite(stepPinB, HIGH);
    digitalWrite(stepPinB, LOW);
  }
  Zaxis.setSpeed( vel_z*250/6);
  Zaxis.runSpeed();
  t += 0.001;
}

void setup() {
  ack_packet[0] = 255;
  ack_packet[1] = 255;
  Serial.begin(115200);
  SerialBT.begin("Unicorn-Z"); //Bluetooth device name
  pinMode(2, OUTPUT); // Bulit-in LED
  pinMode(dirPinB, OUTPUT);
  pinMode(stepPinB, OUTPUT);
//  pinMode(proximityPin, OUTPUT);
  gripper_servo.attach(gripperServoPin);
  timer = timerBegin(0, 80, true); // timer 0, divider 80, count up
  timerAttachInterrupt(timer, &onStepper, true);
  timerAlarmWrite(timer, 1000, true); // call every 1000 ticks (1ms.)
  timerAlarmEnable(timer);
  Zaxis.setMaxSpeed(1000);

//  timer2 = timerBegin(1, 80, true); // timer 0, divider 80, count up
//  timerAttachInterrupt(timer2, &onControl, true);
//  timerAlarmWrite(timer2, 2000, true); // call every 1000 ticks (1ms.)
//  timerAlarmEnable(timer2);

  Serial.println("Start Timer");
}
void loop() 
{
  c1 = 0;
  c2 = 0;
  c3 = 0.734;
  c4 = -0.0489;
  gramma=0.955;
  tf = 10;
//
//    c1 = 0;
//  c2 = 0;
//  c3 = 1.84932;
//  c4 = -0.12328;
//  gramma= 1.33;
//  tf = 10;


//    c1 = 0;
//  c2 = 0;
//  c3 = 1.503;
//  c4 = -0.0501;
//  gramma= 1.5;
//  tf = 20;
  Serial.println(Zaxis.currentPosition());
  
}

void shift_buffer(int n, uint8_t *buffer, int buffer_size){ // Shift buffer to the left N times
    int i, j;
    for(i=0;i<n;i++){
        for(j=0; j<buffer_size-1; j++){
            buffer[j] = buffer[j+1];
        }
    }
}
void setZero() {
  digitalWrite(dirPinA, LOW); // up
  while (digitalRead(limitSwitchPin) == 0) {
    digitalWrite(stepPinA, HIGH); digitalWrite(stepPinA, LOW); delayMicroseconds(2000);
  }
  //Zaxis.setCurrentPosition(400*250/7);
  // Reset remembered position
  stepAPos = 0;
  stepBPos = B_zero;
  stepADes = 0;
  stepBDes = B_zero;
  Serial.printf("Homed!\n");
}
void applyCheckSum(uint8_t *buffer, int length){
    int i;
    uint8_t checksum = 0;
    for(i=2;i<length-1;i++){
        checksum += buffer[i];
    }
    buffer[sizeof(buffer)-1] = ~checksum;
}
