#include <IBusBM.h>
#include <Servo.h>


/*
  Translate IBUS signal to servo
  
  Supports any Arduino board

  Serial port RX/TX connected as follows:
  - RX connected to the ibus servo pin
  - TX left open or connected to the USB/Serial converter to display debug information on your PC

*/

IBusBM IBus; // IBus object
Servo myservo;  // create servo object to control a servo

void setup() {
  // IBUS connected to serial
  IBus.begin(Serial);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  Serial.println("Start IBus2PWM");
}

int saveval=0;

void loop() {
  int val;
  val = IBus.readChannel(0); // get latest value for servo channel 0

  if (saveval != val) {
    Serial.print(val); // display new value
    Serial.print(" ");
    saveval = val;    
    val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo.write(val);                  // sets the servo position according to the scaled value
  }
  
  delay(20);
}

