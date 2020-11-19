#include <ESPmDNS.h>
#include <ESP32Servo.h>

hw_timer_t * timer = NULL;

const char* ssid = "ponyslayer";
const char* password = "unicorn123";
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
int stepAPos = 0, stepBPos = 0;
Servo gripper_servo;
String valueString_gripper = String(5);
String valueString_A = String(5);
String valueString_B = String(5);
int pos1 = 0; int pos2 = 0; //Just buffer for extract values from GET
volatile double t, tf, c1, c2, c3, c4, ti, Q_t, Q_dot_t, theta, gramma, setpoint_z, vel_z, z0;
int stepPin, dirPin, delta;

void setZero();
void stepGo(int motor, int pos);
void stepMaiGo(float C3,float C4, float Tf,float Gramma);

void IRAM_ATTR onTimer(){
  if (t < tf){
        Q_t = c1 + c2 * (t - ti) + c3 * (t - ti) * (t - ti) + c4 * (t - ti) * (t - ti) * (t - ti);
        Q_dot_t = c2 + 2 * c3 * (t - ti) + 3 * c4 * (t - ti) * (t - ti);
        setpoint_z = z0 + (Q_t) * sin(gramma);
        vel_z = (Q_dot_t) * sin(gramma);
        if (vel_z > 0)
        {
          digitalWrite(dirPinA, LOW);
        }
        else
        {
          digitalWrite(dirPinA, HIGH);
        }
        pulseDelay = (224 * 1000000) / (8000 * vel_z);
        digitalWrite(stepPinA, HIGH);
        digitalWrite(stepPinA, LOW);
//        delayMicroseconds(pulseDelay);
      }
      else
      {
        digitalWrite(stepPinA, LOW);
        digitalWrite(stepPinA, LOW);
      }
      t += 0.01;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  /// REAL SETTUP ///
  pinMode(2, OUTPUT); // Bulit-in LED
  pinMode(dirPinA, OUTPUT);
  pinMode(stepPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(stepPinB, OUTPUT);
  pinMode(proximityPin, OUTPUT);
  gripper_servo.attach(gripperServoPin);
  timer = timerBegin(2, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
  Serial.println("Start Timer");
}

void step_drive(int dirPin, int stepPin, int cycle) {
  if (cycle > 0) {
    digitalWrite(dirPin, LOW);
  }
  else {
    digitalWrite(dirPin, HIGH);
  }
  for (int i = 0; i < abs(cycle); i++)
  {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay);
  }
}

void loop() {
  stepMaiGo(24.447,-4.0745,4.0,0.3935);
}
void setZero() {
  digitalWrite(dirPinA, LOW); // UP
  while (digitalRead(limitSwitchPin) == 0) {
    digitalWrite(stepPinA, HIGH); digitalWrite(stepPinA, LOW); delayMicroseconds(pulseDelay);
  }
  // Reset remembered position
  stepAPos = 0;
  stepBPos = 0;
}
void stepGo(int motor, int pos) {
  int stepPin, dirPin, delta;
  switch (motor) {
    case 0:
      stepPin = stepPinA;
      dirPin = dirPinA;
      delta = pos - stepAPos;
      stepAPos = pos;
      break;
    case 1:
      stepPin = stepPinB;
      dirPin = dirPinB;
      delta = pos - stepBPos;
      stepBPos = pos;
      break;
    default:
      break;
  }
  step_drive(dirPin, stepPin, delta);
}
void stepMaiGo(float C3,float C4, float Tf,float Gramma) {
  c1 = 0;
  c2 = 0;
  c3 = C3;
  c4 = C4;
  tf = Tf;
  gramma = Gramma;
  t = 0;
}
