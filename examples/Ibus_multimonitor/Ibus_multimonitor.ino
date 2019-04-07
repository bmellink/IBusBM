#include <IBusBM.h>

/*
  Monitor iBUS signals and show output on the serial monitor usingfor receivers with 2 ibus pins

  On the TGY-IA6B there are 2 ibus pins: one for the servos (only output) and one for the sensors/telemetry
  This example uses a serial port for each ibus line
  
  Requires Arduino board with multiple UARTs (such as ATMEGA 2560 and ESP32)
  - serial0 - monitor output (debug output to PC)
  - serial1 - connected to the servo ibus pin from the receiver
  - serial2 - connected to the sensor/telemetry ibus pin from the receiver

  Hardware connections to setup/test:
  1. Only connect the serial1 (RX1) pin to the ibus servo pin --> you should see the servo values on screen
  2. As 1 and also connect the serial2 (RX2) pin to the ibus sensor/telemetry pin 
     --> you should see the polling counter change (the variable IBus.cnt_poll will increment every 7ms, 
     but the sensor data will not be sent back to your transmitter)
  3. As 1+2 and also connect the serial2 (TX2) pin to the RX2/ibus connection from step 2 using an 1.2k Ohm
     resitor or diode (1N4148 or similar with cathode=white ring of the diode at the side of TX2) 
     --> dummy sensor data should be sent back to the receiver (cnt_sensor also changes value)

  sensor types defined in IBusBM.h:
  
  #define IBUSS_INTV 0 // Internal voltage (in 0.01)
  #define IBUSS_TEMP 1 // Temperature (in 0.1 degrees, where 0=-40'C)
  #define IBUSS_RPM  2 // RPM
  #define IBUSS_EXTV 3 // External voltage (in 0.1V)

*/

IBusBM IBusServo;
IBusBM IBusSensor;

void setup() {
  // initialize serial port for debug
  Serial.begin(115200);

  // iBUS setup
  IBusServo.begin(Serial1);
  // The default RX/TX pins for Serial1 on ESP32 boards are pins 9/10 and they are often not
  // exposed on the printed circuit board. You can change the pin number by replacing the line above with:
  // IBusServo.begin(Serial1, 1, 21, 22);

  IBusSensor.begin(Serial2);

  Serial.println("Start iBUS monitor");

  // adding 2 sensors
  IBusSensor.addSensor(IBUSS_RPM);
  IBusSensor.addSensor(IBUSS_TEMP);
}


#define TEMPBASE 400    // base value for 0'C

// sensor values
uint16_t speed=0;
uint16_t temp=TEMPBASE+200; // start at 20'C

void loop() {
  // show first 8 servo channels
  for (int i=0; i<8 ; i++) {
    Serial.print(IBusServo.readChannel(i));
    Serial.print(" ");
  }
  Serial.print("Cnt=");
  Serial.print(IBusServo.cnt_rec); // count of how many times servo values have been updated
  Serial.print(" POLL=");
  Serial.print(IBusSensor.cnt_poll); // count of polling for sensor existance
  Serial.print(" Sensor=");
  Serial.println(IBusSensor.cnt_sensor); // count of polling for sensor value
  
  IBusSensor.setSensorMeasurement(1,speed);
  speed += 10;                           // increase motor speed by 10 RPM
  IBusSensor.setSensorMeasurement(2,temp++); // increase temperature by 0.1 'C every loop
  delay(500);
}

