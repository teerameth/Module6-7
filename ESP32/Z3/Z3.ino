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
bool homing = false;
void setZero();
void shift_buffer(int n, uint8_t *buffer, int buffer_size);
void applyCheckSum(uint8_t *buffer, int length);

void IRAM_ATTR onStepper(){
  
  reportCnt++;
  if(tA_left > 1)tA_left--; // Time counter in ms.
  if(tB_left > 1)tB_left--;
  if(t<tf){
    theta_t = (((c4*t)+c3)*t+c2)*t+c1;
    theta_dot_t = ((3*c4*t)+c3*2)*t + c2;
    setpoint_z = (theta_t)*sin(gramma);
    vel_z = (theta_dot_t)*sin(gramma);
    //t_stepACnt++;
  }
  else{
    Zaxis.moveTo((long)(setpoint_z*250/7));
  }
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
  Zaxis.setSpeed( -vel_z*250/6);
  if(!homing)Zaxis.runSpeed();
  t += 0.001;
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
void loop() {

  if(reportCnt > 200){
    reportCnt = 0;
    digitalWrite(2, led_state);
    led_state = !led_state;
    Serial.printf("delta %d\t:\t%f\n", delta,setpoint_z);
//    Serial.printf("tA_left: %d, tB_left: %d\n", tA_left, tB_left);
//    Serial.printf("Vel_Z = %f, t = %f, tf=%f\n", vel_z, t, tf);
  }
  
  if(SerialBT.available()){
    byte_in = SerialBT.read();
    stack[stackSize] = byte_in;
    stackSize++;
//    Serial.printf("%d\n", byte_in);
  }
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
//                    Serial.printf("StartIndex %d\nPacketLength %d\nChecksum %d\nChecksumReal %d\n", startIndex, packetLength, checksum, stack[startIndex+packetLength+2]);
                    if(stack[startIndex+packetLength+2] == checksum){ // Checksum correct
//                      Serial.println("Checksum Correct\n");
                        uart_state = 3;
                        break;
                    }
                    else{
                        // Checksum error
//                        for(int i=0;i<sizeof(stack);i++){
//                          Serial.printf("%d ", stack[i]); 
//                        }
//                        Serial.println("Checksum Error!");
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
                        case 2: //Read(Z) {255, 255, 4, 2, cur_pos_z/256, cur_pos_z%256, 0}
                            ack_packet[2] = 4;
                            ack_packet[3] = 2;
                            ack_packet[4] = stepAPos/256; // Z-high byte
                            ack_packet[5] = stepAPos%256; // Z-low byte
                            applyCheckSum(ack_packet, 7);
                            SerialBT.write(ack_packet, 7);
                            Serial.printf("Z_pos = %d", stepAPos);
                            break;
                        case 3: //Write (z) (a) {255, 255, 5, 3, 0/1, high_byte, low_byte, checkSum} or {255, 255, 7, 3, 2/3, high_byte, low_byte, t_1, t_2, checkSum}
                            t_stepACnt = 0;
                            t_stepBCnt = 0;
                            switch(stack[startIndex+4]){
                              case 0: // Z-axis
                                stepADes = int(stack[startIndex+5]*256 + stack[startIndex+6]);
                                Serial.printf("Z goto %d\n", int(stack[startIndex+5]*256 + stack[startIndex+6]));
                                break;
                              case 1: // Alpha
                                stepBDes = int(stack[startIndex+5]*256 + stack[startIndex+6]);
                                Serial.printf("A goto %d\n", int(stack[startIndex+5]*256 + stack[startIndex+6]));
                                break;
                              case 2: // Z-axis with time
                                stepADes = int(stack[startIndex+5]*256 + stack[startIndex+6]);
                                tA_left = int(stack[startIndex+7]*256 + stack[startIndex+8]); // time in ms.
                                Serial.printf("Z goto %d in %d miilisec\n", int(stack[startIndex+5]*256 + stack[startIndex+6]), int(stack[startIndex+7]*256 + stack[startIndex+8]));
                                break;
                              case 3: // Alpha with time
                                stepBDes = int(stack[startIndex+5]*256 + stack[startIndex+6]);
                                tB_left = int(stack[startIndex+7]*256 + stack[startIndex+8]); // time in ms.
                                Serial.printf("A goto %d in %d miilisec\n", int(stack[startIndex+5]*256 + stack[startIndex+6]), int(stack[startIndex+7]*256 + stack[startIndex+8]));
                                break;
                              default:
                                break;
                            }
                            
                            break;
                        case 4: //Write Trajectory {255, 255, n+2, 4, c3_0, c3_1, c3_2, c3_3, c4_0, c4_1, c4_2, c4_3, gamma_0, gamma_1, gamma_2, gamma_3, t_1, t_2, t_3, checkSum}
                            c3 = ((stack[startIndex+4]==0)?1.0:-1.0)*(((float)stack[startIndex+5])+((float)stack[startIndex+6])/100+((float)stack[startIndex+7])/10000);
                            c4 = ((stack[startIndex+8]==0)?1.0:-1.0)*(((float)stack[startIndex+9])+((float)stack[startIndex+10])/100+((float)stack[startIndex+11])/10000);
                            gramma = ((stack[startIndex+12]==0)?1.0:-1.0)*(((float)stack[startIndex+13])+((float)stack[startIndex+14])/100+((float)stack[startIndex+15])/10000);
                            tf = ((float)stack[startIndex+16]) + ((float)stack[startIndex+17])/100 + ((float)stack[startIndex+18])/10000;
                            t = 0;
                            break;
                        case 5: // Home {255, 255, 3, 5, 0, checksum}
                            Serial.printf("Set Home\n");
                            homing = true;
                            setZero();
                            homing = false;
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
  digitalWrite(dirPinA, LOW); // up
  while (digitalRead(limitSwitchPin) == 0) {
    digitalWrite(stepPinA, HIGH); digitalWrite(stepPinA, LOW); delayMicroseconds(1000);
  }
  //Zaxis.setCurrentPosition(400*250/7);
  // Reset remembered position
  stepAPos = 0;
  stepBPos = B_zero;
  stepADes = 0;
  stepBDes = B_zero;
  Serial.printf("Homed!\n");
  ack_packet[2] = 3;
  ack_packet[3] = 5;
  ack_packet[4] = 1;
  ack_packet[5] = 246; // checksum
  SerialBT.write(ack_packet, 6);
}
void applyCheckSum(uint8_t *buffer, int length){
    int i;
    uint8_t checksum = 0;
    for(i=2;i<length-1;i++){
        checksum += buffer[i];
    }
    buffer[sizeof(buffer)-1] = ~checksum;
}
