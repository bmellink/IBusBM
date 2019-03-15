#include <IBusBM.h>
#include <Servo.h>


/*
  Translate iBUS signal to servo
  
  Supports any Arduino board where serial0 is available, such as Nano, Mini. Micro.
  Please note the serial port on these boards is also used during programming, so you need to
  setup your wiring after programming the sketch.
  
  For boards where serial0 is connected to the onboard USB port (such as MEGA and UNO) you need
  to disconnect the RX line from the receiver during programming. 
  Alternatively you can change the code below to use another serial port.

  Please use 5V boards only.

  Serial port RX/TX connected as follows:
  - RX connected to the iBUS servo pin (disconnect during programming on MEGA and UNO boards!)
  - TX left open or connected to an USB/Serial converter to display debug information on your PC (set baud rate to 115200).  

*/

IBusBM IBus; // IBus object
Servo myservo;  // create servo object to control a servo

void setup() {
  // iBUS connected to serial
  IBus.begin(Serial);
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

