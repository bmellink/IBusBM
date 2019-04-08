#include <IBusBM.h>

/*
  Simulate two sensor and send information back over the iBUS to the receiver (and back to transmitter
  as telemetry).
  
  Requires Arduino board with multiple UARTs (such as ATMEGA 2560, Micro or ESP32)
  - serial0 - monitor output (debug output to PC, this is through the build-in USB)
  - serial1 - connected RX of the serial port to the ibus receiver pin
    Connect the (TX) pin also to the RX/ibus connection using an 1.2k Ohm reistor or 1N4148 diode
    (cathode=white ring of the diode at the side of TX2) 

  sensor types defined in IBusBM.h:
  
  #define IBUSS_INTV 0 // Internal voltage (in 0.01)
  #define IBUSS_TEMP 1 // Temperature (in 0.1 degrees, where 0=-40'C)
  #define IBUSS_RPM  2 // RPM
  #define IBUSS_EXTV 3 // External voltage (in 0.1V)

*/

IBusBM IBus; 

void setup() {
  // initialize serial port for debug
  Serial.begin(115200);

  // iBUS connected to serial1
  IBus.begin(Serial1);
  // The default RX/TX pins for Serial1 on ESP32 boards are pins 9/10 and they are sometimes not
  // exposed on the printed circuit board. You can use Serial2 (for which the pins are often available) or
  // you can change the pin number by replacing the line above with:
  // IBusServo.begin(Serial1, 1, 21, 22);

  Serial.println("Start iBUS sensor");

  // adding 2 sensors
  IBus.addSensor(IBUSS_RPM);
  IBus.addSensor(IBUSS_TEMP);
}


#define TEMPBASE 400    // base value for 0'C

// sensor values
uint16_t speed=0;
uint16_t temp=TEMPBASE+200; // start at 20'C

void loop() {
  IBus.setSensorMeasurement(1,speed);
  speed += 10;                           // increase motor speed by 10 RPM
  IBus.setSensorMeasurement(2,temp++); // increase temperature by 0.1 'C every loop
  Serial.print("Speed=");
  Serial.print(speed);
  Serial.print(" Temp=");
  Serial.println((temp-TEMPBASE)/10.);
  delay(500);
}

