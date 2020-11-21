#include "BluetoothSerial.h"
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#define STEP_PER_ROUND 800
#define B_zero (int)(STEP_PER_ROUND/2)
BluetoothSerial SerialBT;
hw_timer_t * timer = NULL;
// Note: 200cycles : 5mm.
const int dirPinA = 25;
const int stepPinA = 26;
const int dirPinB = 32;
const int stepPinB = 33;
const int proximityPin = 27; // Active LOW
const int gripperServoPin = 14;
const int limitSwitchPin = 12; // Homing switch
float pulseDelay; // 1000 for 28byj-48, 500 for NEMA-17
int gripper_pos = 0;
int stepAPos = 0, stepBPos = B_zero, stepADes = 0, stepBDes = 0;
Servo gripper_servo;
volatile long unsigned int t_stepA, t_stepB, t_stepACnt, t_stepBCnt, tA_left, tB_left;
int stepPin, dirPin, delta;
int deltaA, deltaB;
uint8_t stack[40], ack_packet[20];
uint8_t stackSize=0, startIndex, packetLength, uart_state = 0, checksum;
char byte_in;

void setZero();
void stepMaiGo(float C3,float C4, float Tf,float Gramma);
void shift_buffer(int n, uint8_t *buffer, int buffer_size);
void applyCheckSum(uint8_t *buffer, int length);

void IRAM_ATTR onStepper(){
  if(tA_left > 0)tA_left--; // Time counter in ms.
  if(tB_left > 0)tB_left--;
  deltaA = stepADes - stepAPos;
  if(deltaA){
    t_stepA = tA_left/abs(deltaA);
    t_stepACnt++;
    if(t_stepACnt > t_stepA){
      t_stepACnt -= t_stepA;
      if(deltaA > 0){
        digitalWrite(dirPinA, LOW);
        stepAPos++;
      }
      else{
        digitalWrite(dirPinA, HIGH);
        stepAPos--;
      }
      digitalWrite(stepPinA, HIGH);
      digitalWrite(stepPinA, LOW);
    }
  }
  deltaB = stepBDes - stepBPos;
  if(deltaB){
    t_stepB = tB_left/abs(deltaB);
    t_stepBCnt++;
    if(t_stepBCnt > t_stepB){
      t_stepBCnt -= t_stepB;
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
  }
}

void setup() {
  ack_packet[0] = 255;
  ack_packet[1] = 255;
  Serial.begin(115200);
  SerialBT.begin("Unicorn-Z"); //Bluetooth device name
  pinMode(2, OUTPUT); // Bulit-in LED
  pinMode(dirPinA, OUTPUT);
  pinMode(stepPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(stepPinB, OUTPUT);
  pinMode(proximityPin, OUTPUT);
  gripper_servo.attach(gripperServoPin);
  timer = timerBegin(0, 240, true); // timer 0, divider 240, count up
  timerAttachInterrupt(timer, &onStepper, true);
  timerAlarmWrite(timer, 1000, true); // call every 1000 ticks (1ms.)
  timerAlarmEnable(timer);
  Serial.println("Start Timer");
}
void loop() {
  

  if(SerialBT.available()){
    byte_in = SerialBT.read();
    stack[stackSize] = byte_in;
    stackSize++;
//    Serial.printf("%d\n", byte_in);
  }
//  stepMaiGo(24.447,-4.0745,4.0,0.3935);
  if (stackSize > 2) // Time to check data
        {
            switch(uart_state){
                case 0:
                    for(int i=0;i<stackSize-1;i++){
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
                    for(int i=startIndex+2;i<startIndex+packetLength+2;i++){checksum+=stack[i];}
                    checksum = ~checksum;
                    if(stack[startIndex+packetLength+2] == checksum){ // Checksum correct
//                        Serial.println("Checksum Passed!\n");
                        uart_state = 3;
                        break;
                    }
                    else{
                        // Checksum error
                        Serial.println("Checksum Error!");
                        shift_buffer(startIndex+1, stack, sizeof(stack));
                        stackSize -= startIndex+1;
                        break;
                    }
                case 3: // Call function
                    switch(stack[startIndex+3]){ //Instruction (ParameterN is at startIndex+3+N)
                        case 1: //Ping {255, 255, 3, 1, rand, checksum}
                            ack_packet[2] = stack[startIndex+2];
                            ack_packet[3] = stack[startIndex+3];
                            ack_packet[4] = stack[startIndex+4];
                            ack_packet[5] = stack[startIndex+5];
                            SerialBT.write(ack_packet, 6);
                            
                            Serial.println("Ping");
                            break;
                        case 2: //Read(Z) {255, 255, 4, 5, cur_pos_z/255, cur_pos_z%255, 0}
                            ack_packet[2] = 6;
                            ack_packet[3] = 2;
                            ack_packet[4] = stepAPos/256; // X-high byte
                            ack_packet[5] = stepAPos%256; // X-low byte
                            applyCheckSum(ack_packet, 9);
                            SerialBT.write(ack_packet, 7);
                            Serial.printf("Z_pos = %d", stepAPos);
                            break;
                        case 3: //Write (z) (a) {255, 255, 5, 3, 0/1, high_byte, low_byte, checkSum} or {255, 255, 7, 3, 0/1, high_byte, low_byte, t_1, t_2, checkSum}
                            switch(stack[startIndex+4]){
                              case 0: // Z-axis
                                stepADes = int(stack[startIndex+5]*256 + stack[startIndex+6]);
                                break;
                              case 1: // Alpha
                                stepBDes = int(stack[startIndex+5]*256 + stack[startIndex+6]);
                                break;
                              case 2: // Z-axis with time
                                stepADes = int(stack[startIndex+5]*256 + stack[startIndex+6]);
                                tA_left = stack[startIndex+7]*256 + stack[startIndex+8]; // time in ms.
                                break;
                              case 3: // Alpha with time
                                stepBDes = int(stack[startIndex+5]*256 + stack[startIndex+6]);
                                tB_left = stack[startIndex+7]*256 + stack[startIndex+8]; // time in ms.
                                break;
                              default:
                                break;
                            }
                            Serial.printf("Z goto %d", int(stack[startIndex+5]*256 + stack[startIndex+6]));
                            break;
                        case 4: //Write Trajectory {255, 255, n+2, 4, c3_0, c3_1, c3_2, c3_3, c4_0, c4_1, c4_2, c4_3, gamma_0, gamma_1, gamma_2, gamma_3, t_1, t_2, t_3, checkSum}
//                            stepMaiGo(((stack[startIndex+4]==0)?1.0:-1.0)*(((float)stack[startIndex+5])+((float)stack[startIndex+6])/100+((float)stack[startIndex+7])/10000),
//                            ((stack[startIndex+8]==0)?1.0:-1.0)*(((float)stack[startIndex+9])+((float)stack[startIndex+10])/100+((float)stack[startIndex+11])/10000),
//                            ((stack[startIndex+12]==0)?1.0:-1.0)*(((float)stack[startIndex+13])+((float)stack[startIndex+14])/100+((float)stack[startIndex+15])/10000),
//                            (((float)stack[startIndex+16])+((float)stack[startIndex+17])/100+((float)stack[startIndex+18])/10000));
                            break;
                        case 5: // Home {255, 255, 3, 5, 0, checksum}
                            Serial.printf("Set Home!");
                            setZero();
                            break;
                        case 6: // Gripper servo {255, 255, 3, 6, servoPos, checksum}
                            gripper_servo.write(stack[startIndex+4]);
                            Serial.printf("Servo: %d\n", stack[startIndex+4]);
                        default:
                            break;
                    }
                    shift_buffer(startIndex+packetLength+3, stack, sizeof(stack));//Clear used stack
                    stackSize -= startIndex+packetLength+3;
                    uart_state = 0;
            }
        }
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
  digitalWrite(dirPinA, HIGH); // DOWN
  while (digitalRead(limitSwitchPin) == 0) {
    digitalWrite(stepPinA, HIGH); digitalWrite(stepPinA, LOW); delayMicroseconds(2000);
  }
  // Reset remembered position
  stepAPos = 0;
  stepBPos = B_zero;
  Serial.printf("Homed!");
}
//void stepMaiGo(float C3,float C4, float Tf,float Gramma) {
//  c1 = 0;
//  c2 = 0;
//  c3 = C3;
//  c4 = C4;
//  tf = Tf;
//  gramma = Gramma;
//  t = 0;
//}
void applyCheckSum(uint8_t *buffer, int length){
    int i;
    uint8_t checksum = 0;
    for(i=2;i<length-1;i++){
        checksum += buffer[i];
    }
    buffer[sizeof(buffer)-1] = ~checksum;
}
