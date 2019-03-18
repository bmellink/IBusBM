#include <IBusBM.h>
#include <Servo.h>


/*
  Translate iBUS signal to servo
  
  Supports any Arduino board where serial0 is available. 
 
  For boards where serial0 is connected to the onboard USB port (such as MEGA, UNO and micro) you need
  to disconnect the RX line from the receiver during programming. 

  Alternatively you can change the code below to use another serial port.

  Please use 5V boards only.

  Serial port RX/TX connected as follows:
  - RX connected to the iBUS servo pin (disconnect during programming on MEGA, UNO and micro boards!)
  - TX left open or for Arduino boards without an onboard USB controler - connect to an 
    USB/Serial converter to display debug information on your PC (set baud rate to 115200).  

*/

IBusBM IBus; // IBus object
Servo myservo;  // create servo object to control a servo

void setup() {
  // Serial.begin(115200);   // remove comment from this line if you change the Serial port in the next line

  IBus.begin(Serial);    // iBUS connected to Serial0 - change to Serial1 or Serial2 port when required

  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  Serial.println("Start IBus2PWM");
}

int saveval=0;

void loop() {
  int val;
  val = IBus.readChannel(0); // get latest value for servo channel 1

  if (saveval != val) {
    Serial.println(val); // display new value in microseconds on PC
    saveval = val;    
    myservo.writeMicroseconds(val);   // sets the servo position 
  }
  
  delay(100);
}

