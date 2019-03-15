#include <IBusBM.h>

/*
  Monitor iBUS signals and show output on the serial monitor for receivers with a single ibus pin
  such as the Flysky FS-iA8X (from specs, not tested yet). The TGY-IA6C also has
  one iBUS pin but only supports servo control signals and does not support external telemetry sensors.
  
  Requires Arduino board with multiple UARTs (such as ATMEGA 2560)
  - serial0 - monitor output (debug output to PC)
  - serial1 - connected to the ibus receiver pin

  Hardware connections to setup/test if you have a receiver with 1 ibus pins:
  1. Only connect the serial1 (RX1) pin to the ibus pin of the receiver 
     --> you should see the servo values on screen
  2. Connect the serial1 (TX1) pin also to the RX1/ibus connection using a diode
     (cathode=white ring of the diode at the side of TX2) 
     --> dummy sensor data should be sent back to the receiver (cnt_sensor also changes value)

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
  
  Serial.println("Start iBUS monitor");

  // adding 2 sensors
  IBus.addSensor(IBUSS_RPM);
  IBus.addSensor(IBUSS_TEMP);
}


#define TEMPBASE 400    // base value for 0'C

// sensor values
uint16_t speed=0;
uint16_t temp=TEMPBASE+200; // start at 20'C

void loop() {
  // show first 8 servo channels
  for (int i=0; i<8 ; i++) {
    Serial.print(IBus.readChannel(i));
    Serial.print(" ");
  }
  Serial.print("Cnt=");
  Serial.print(IBus.cnt_rec); // count of how many times servo values have been updated
  Serial.print(" POLL=");
  Serial.print(IBus.cnt_poll); // count of polling for sensor existance
  Serial.print(" Sensor=");
  Serial.println(IBus.cnt_sensor); // count of polling for sensor value
  
  IBus.setSensorMeasurement(1,speed);
  speed += 10;                           // increase motor speed by 10 RPM
  IBus.setSensorMeasurement(2,temp++); // increase temperature by 0.1 'C every loop
  delay(500);
}

