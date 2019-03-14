#include <IBusBM.h>

/*
  Monitor IBUS signals and show output on the serial monitor
  
  Requires Arduino board with multiple UARTs (such as ATMEGA 2560)
  - serial0 - monitor output
  - serial1 - connected to the ibus

  On the TGY-IA6B there are 2 ibus pins: one for the servos (only output) and one for the sensors/telemetry
  (which uses a half duplex protocol to switch between output and input to poll the sensors)

  You have 4 options:
  - If you connect the serial1 (RX1) pin to the ibus servo pin you will only see the servo values on screen
  - If you connect the serial1 (RX1) pin to the ibus sensor/telemetry pin, you will only see the polling for sensors
    (the variable IBus.cnt_poll will increment every 7ms, but the data will not be sent back to your transmitter)
  - If you connect both the ibus servo and sensor pin to the RX1 pin of the serial1 using a diode and resistor as described in
    https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry you will see both servo data and
    polling for sensor data. However the sensor data is still not sent back to the transmitter
  - If you add another diode between the TX1 pin of the Arduino and the telemetry ibus pin of the receiver (cathode=white ring
    of the diode at the side of TX1) you will also see the sensor data being send back to the receiver (cnt_sensor 
    also increases in value)

  Alternatively you can also use two different UARTS for servo and telemetry data by defining two different IBus objects:
    IBusBM IBusServo;
    IBusBM IBusSensor;
  assigning them to the right Serial1/Serial2 ports in setup()

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

  // IBUS connected to serial1
  IBus.begin(Serial1);
  
  Serial.println("Start IBUS monitor");

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
    Serial.print(IBus.readChannel(i), HEX);
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

