// Note: 200cycles : 5mm.
const int stepPin = 12;
const int dirPin = 14;
void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
}
void step_drive(bool dir, float cycle){
  if(dir)digitalWrite(dirPin,HIGH);
  else digitalWrite(dirPin,LOW);
  for(int x = 0; x < cycle; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(1000); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(1000); 
  }
}
void loop() {
  step_drive(0, 200);
  delay(100); // One second delay
  step_drive(0, 200);
  delay(100);
}
