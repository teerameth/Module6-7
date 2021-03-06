#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
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
WiFiServer server(80);

void setZero();
void stepGo(int motor, int pos);
void stepMaiGo(float C3,float C4, float Tf,float Gramma);
void IRAM_ATTR onTimer(){
  if (t < tf){
        Q_t = c1 + c2 * (t - ti) + c3 * (t - ti) * (t - ti) + c4 * (t - ti) * (t - ti) * (t - ti);
        Q_dot_t = c2 + 2 * c3 * (t - ti) + 3 * c4 * (t - ti) * (t - ti);
        setpoint_z = z0 + (Q_t) * sin(gramma);
        vel_z = (Q_dot_t) * sin(gramma);
        delta = ((int)(setpoint_z*500.0/14.0)) - stepAPos;
        if (delta > 0)
        {
          digitalWrite(dirPinA, LOW);
          stepAPos++;
        }
        else
        {
          digitalWrite(dirPinA, HIGH);
          stepAPos--;
        }
        pulseDelay = (setpoint_z)*1000000 / (vel_z);
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
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); \

  /// REAL SETTUP ///
  pinMode(2, OUTPUT); // Bulit-in LED
  pinMode(dirPinA, OUTPUT);
  pinMode(stepPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(stepPinB, OUTPUT);
  pinMode(proximityPin, OUTPUT);
  gripper_servo.attach(gripperServoPin);
  server.begin();
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
  ArduinoOTA.handle();
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
//            // Display the HTML web page
//            client.println("<!DOCTYPE html><html>");
//            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
//            client.println("<link rel=\"icon\" href=\"data:,\">");
//            // CSS to style the on/off buttons
//            // Feel free to change the background-color and font-size attributes to fit your preferences
//            client.println("<style>body { text-align: center; font-family: \"Trebuchet MS\", Arial; margin-left:auto; margin-right:auto;}");
//            client.println(".slider { width: 300px; }</style>");
//            client.println("<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js\"></script>");
//
//            // the content of the HTTP response follows the header:
//            client.print("<a href=\"/R\">SET ZERO</a><br>");
//            client.print("<a href=\"/H\">ON</a><br>");
//            client.print("<a href=\"/L\">OFF</a><br>");
//            client.print("<a href=\"/U\">UP</a><br>");
//            client.print("<a href=\"/D\">DOWN</a><br>");
//            client.print("<a href=\"/CW\">CLOCKWISE</a><br>");
//            client.print("<a href=\"/CCW\">COUNTERCLOCKWISE</a><br>");
//            /// Servo Slider ///
//            client.println("</head><body><h1>Servo</h1>");
//            client.println("<p>Position: <span id=\"servoPos\"></span></p>");
//            client.println("<input type=\"range\" min=\"0\" max=\"180\" class=\"slider\" id=\"servoSlider\" onchange=\"servo(this.value)\" value=\"" + valueString_gripper + "\"/>");
//            // Script
//            client.println("<script>var slider = document.getElementById(\"servoSlider\");");
//            client.println("var servoP = document.getElementById(\"servoPos\"); servoP.innerHTML = slider.value;");
//            client.println("slider.oninput = function() { slider.value = this.value; servoP.innerHTML = this.value; }");
//            client.println("$.ajaxSetup({timeout:1000}); function servo(pos) { ");
//            client.println("$.get(\"/?value=\" + pos + \"&\"); {Connection: close};}</script>");
//            /////////////// Stepper Slider ///////////////
//            /// Z-Linear ///
//            client.println("</head><body><h1>Z-Linear(Motor A)</h1>");
//            client.println("<p>Position: <span id=\"stepA_pos\"></span></p>");
//            client.println("<input type=\"range\" min=\"0\" max=\"180\" class=\"slider\" id=\"stepA_slider\" onchange=\"servo(this.value)\" value=\"" + valueString_A + "\"/>");
//            // Script
//            client.println("<script>var sliderA = document.getElementById(\"stepA_slider\");");
//            client.println("var stepA_P = document.getElementById(\"stepA_pos\"); stepA_P.innerHTML = sliderA.value;");
//            client.println("sliderA.oninput = function() { sliderA.value = this.value; stepA_P.innerHTML = this.value; }");
//            client.println("$.ajaxSetup({timeout:1000}); function servoA(pos) { ");
//            client.println("$.get(\"/?valueA=\" + pos + \"&\"); {Connection: close};}</script>");
//            /// Z-Rotation ///
//            client.println("</head><body><h1>Z-Rotation (Motor B)</h1>");
//            client.println("<p>Position: <span id=\"stepB_pos\"></span></p>");
//            client.println("<input type=\"range\" min=\"0\" max=\"180\" class=\"slider\" id=\"stepB_slider\" onchange=\"servo(this.value)\" value=\"" + valueString_B + "\"/>");
//            // Script
//            client.println("<script>var sliderB = document.getElementById(\"stepB_slider\");");
//            client.println("var stepB_P = document.getElementById(\"stepB_pos\"); stepB_P.innerHTML = slider.value;");
//            client.println("sliderB.oninput = function() { sliderB.value = this.value; stepB_P.innerHTML = this.value; }");
//            client.println("$.ajaxSetup({timeout:1000}); function servoB(pos) { ");
//            client.println("$.get(\"/?valueB=\" + pos + \"&\"); {Connection: close};}</script>");
//            client.println("</body></html>");
//            client.println(); // The HTTP response ends with another blank line:
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /R")) { // Set home of vertical axis
            setZero();
        }
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(2, HIGH);
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(2, LOW);
        }
        if (currentLine.endsWith("GET /U")) {
          step_drive(dirPinA, stepPinA, 200);
          stepAPos += 200;
        }
        if (currentLine.endsWith("GET /D")) {
          step_drive(dirPinA, stepPinA, -200);
          stepAPos -= 200;
        }
        if (currentLine.endsWith("GET /CW")) {
          step_drive(dirPinB, stepPinB, 100);
          stepBPos += 100;
        }
        if (currentLine.endsWith("GET /CCW")) {
          step_drive(dirPinB, stepPinB, -100);
          stepBPos -= 100;
        }
        //GET /?value=180& HTTP/1.1 http://192.168.1.135/?value=180&
        if (currentLine.indexOf("GET /?value=") >= 0) {
          pos1 = currentLine.indexOf('=');
          pos2 = currentLine.indexOf('&');
          valueString_gripper = currentLine.substring(pos1 + 1, pos2);
          //Rotate the servo
          gripper_servo.write(valueString_gripper.toInt());
        }
        if (currentLine.indexOf("GET /?valueA=") >= 0) {
          pos1 = currentLine.indexOf('=');
          pos2 = currentLine.indexOf('&');
          valueString_A = currentLine.substring(pos1 + 1, pos2);
          stepGo(0, valueString_A.toInt());
        }
        if (currentLine.indexOf("GET /?valueB=") >= 0) {
          pos1 = currentLine.indexOf('=');
          pos2 = currentLine.indexOf('&');
          valueString_B = currentLine.substring(pos1 + 1, pos2);
          stepGo(1, valueString_B.toInt());
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }

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
