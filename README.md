# Arduino RC IBus protocol handler
Arduino library for Flysky/Turnigy RC iBUS protocol - servo (receive) and sensors/telemetry (send) using hardware UART.

The iBUS protocol is a half-duplex protocol developed by Flysky to control multiple servos and motors using a single digital line. The values received for each servo channel are between 1000 (hex eE8) and 2000 (hex 7D0) with neutral sub trim setting, which corresponds with the pulse width in microseconds for most servos.

The protocol can also connect sensors to send back telemetry information to a RC transceiver. Currently the protocol only supports 3 sensor types: Voltage, Temperature and Motor speed (RPM). You can define up to 10 sensors using this library.

This library was written and tested for the TGY-IA6B receiver and should work for other receivers too.
The TGY-IA6B has 2 iBUS pins: one for the servos (only output) and one for the sensors/telemetry
(which uses a half-duplex protocol to switch between output and input to poll for sensor data).

Receivers with one iBUS pin typically send servo commands over the iBUS line, but do not always poll external sensors. The TGY-iA6C for instance only sends servo data over the iBUS and is only able to send back internal telemetry data (such as Rssi and voltage) and the voltage measurement of the B-DET connection. The specs of the Flysky X6B and Flysky FS-iA8X are unclear if telemetry sensors is supported over iBUS.

## Getting Started

To install this library use the **Clone or download > Download ZIP** button on the repository home page and then install the library in your Arduino environment using **Sketch > Include Library > Add .ZIP Library...**

This library is for AVR based Arduino boards only (Arduino MEGA, UNO, Nano, Micro, etc.).

### Prerequisites

The iBUS library requires a dedicated hardware serial (UART) port on the Arduino board. If your board only has one UART port you can still use that port for serial debug communication with your PC as long as you plan to use servo output mode only (the baud rate will be fixed at 115200 baud and you should only attach the UART TX pin to the USB-Serial converter). The ATMEGA boards typically have more than one UART port.

You have three options:
- If you plan to **only use servo output** iBUS data in your sketch, you only need to connect the iBUS servo output pin from your receiver to the RX pin of the allocated UART. The TX pin of the UART will not be used by the library as there is no information sent back to the RC receiver.
- If you plan to **only provide sensor** iBUS data from your sketch, you will need to connect the iBUS sensor pin to both the RX and TX pin of the allocated UART. You need to include a diode (such as 1N4148) between the Arduino TX pin and the wire between the iBUS pin and the Arduino RX pin (cathode/solid ring of diode connected at Arduino TX pin) to handle the half-duplex protocol over the single iBUS wire.
- If you plan to **use both servo output and sensor data** in your sketch, your should use two different UART ports on your Arduino board.

For more information on the iBUS protocol, see (https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry). (please skip over the first part of the article how to combine the servo output and sensor data pins of the TGY receiver as it is more reliable to use two different UART ports on your Arduino if you need both servo and sensor data in your sketch).

### Example wiring

![Wiring with MEGA 2560](wiring.png?raw=true "Wiring with MEGA 2560")

### Example code for servo output only

This example is for any Arduino board.

```
#include <IBusBM.h>
#include <Servo.h>

IBusBM IBus;    // IBus object
Servo myservo;  // create servo object to control a servo

void setup() {
  IBus.begin(Serial);    // iBUS object connected to serial0 RX/RX0 pin
  myservo.attach(9);     // attaches the servo on pin 9 to the servo object
}

void loop() {
  int val;
  val = IBus.readChannel(0); // get latest value for servo channel 1
  myservo.writeMicroseconds(val);   // sets the servo position 
  delay(20);
}

```

You can support more servos by simply adding more Servo objects:
```
#include <IBusBM.h>
#include <Servo.h>

IBusBM IBus;    // IBus object
Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo

void setup() {
  IBus.begin(Serial);    // iBUS object connected to serial0 RX/RX0
  myservo1.attach(8);     // attaches the servo on pin 8 to the servo1 object
  myservo2.attach(9);     // attaches the servo on pin 9 to the servo2 object
}

void loop() {
  int val;
  val = IBus.readChannel(0); // get latest value for servo channel 1
  myservo1.writeMicroseconds(val);   // sets the servo position 
  val = IBus.readChannel(1); // get latest value for servo channel 2
  myservo2.writeMicroseconds(val);   // sets the servo position 
  delay(20);
}

```

### Example code for combined servo output and sensor input using two UART ports
This example is for the MEGA 2560 board

```
#include <IBusBM.h>

IBusBM IBusServo;
IBusBM IBusSensor;

#define TEMPBASE 400    // base value for temperature is -40'C

// sensor values
uint16_t speed=0;
uint16_t temp=TEMPBASE+200; // start at 20'C

void setup() {
  // initialize serial 0 port for debug messages on your PC
  Serial.begin(115200);

  // iBUS servo connected to RX of serial1 port
  IBusServo.begin(Serial1);

  // iBUS sensor connected to RX + TX of serial2 port (1N4148 diode included in TX line - cathode connected to TX)
  IBusSensor.begin(Serial2);
  
  Serial.println("Start iBUS");

  // adding 2 sensors to generate some dummy data
  IBus.addSensor(iBUSS_RPM);
  IBus.addSensor(iBUSS_TEMP);
}

void loop() {
  // show first 8 servo channels
  for (int i=0; i<8 ; i++) {
    Serial.print(IBusServo.readChannel(i), HEX);
    Serial.print(" ");
  }
  Serial.print(" ServoCnt=");
  Serial.print(IBusServo.cnt_rec); // count of how many times servo values have been updated
  Serial.print(" SensorCnt=");
  Serial.println(IBusSensor.cnt_sensor); // count of how many times sensor values have been sent back

  IBus.setSensorMeasurement(1,speed);
  speed +=10;                           // increase motor speed by 10 RPM
  IBus.setSensorMeasurement(2,temp++);  // increase temperature by 0.1 'C every loop

  delay(500);
}

```

## Class member functions and data members

The IBusBM class exposes the following functions:

- void begin(HardwareSerial& serial); // initialization with defined hardware serial port
- uint16_t readChannel(uint8_t channelNr); // read the value of servo channel 0..9 corresponding with servo channels 1..10
- uint8_t addSensor(uint8_t type); // defines new sensor of type "type", returns sensor number (first is number 1)
- void setSensorMeasurement(uint8_t adr, uint16_t value); // set value of sensor number adr to value (first sensor is number 1)

The IBusBM class exposes the following counters. Counters are 1 byte (value 0..255) and values can be used by your code to understand if new data is available. 

- uint8_t cnt_rec; // count received number of servo messages from receiver
- uint8_t cnt_poll; // count received number of sensor poll messages
- uint8_t cnt_sensor; // count times a sensor value has been sent back

Counters can also be used to debug the hardware connections between the receiver and the Arduino board: If at least one sensor is defined, the RX pin will receive sensor poll messages. If the sensor is not able to "talk back" to the receiver the receiver will try to establish contact with the sensor every 7ms and the cnt_poll counter will increment. Only if the TX pin is correctly connected to the receiver, the cnt_sensor counter will increment and the cnt_poll value will stay the same.

## Failsafe

Failsafe defines what to do if the transmitter looses the connection with the receiver. You can not use the cnt_rec counter for failsafe as the receiver will continue to send servo data over the iBUS even if the connection with the transmitter is lost. You can implement failsafe by defining a failsafe value for a given servo channel in your transmitter (this only works for receivers that support failsafe mode). You can then read the channel value using readChannel() and test for the failsafe value.

## Sensor types

The following sensor types are defined in IBusBM.h:

```
#define iBUSS_INTV 0 // Internal voltage (in 0.01, so 500=5V)
#define iBUSS_TEMP 1 // Temperature (in 0.1 degrees Celcius, where 0=-40'C and 400=0 'C')
#define iBUSS_RPM  2 // RPM
#define iBUSS_EXTV 3 // External voltage (in 0.01, so 510=5.1V)
```

## Background processing and Interrupts

IBusBM runs in the background using interrupts to ensure your code does not interfere with the iBUS timing. Timer0 is used by the core libraries on all Arduino boards to keep track of time for commands line millis() and delay(). IBusBM defines an additional interrupt on Timer 0 (the compare interrupt - TIMER0_COMPA_vect) to trigger the main process. Be sure not to change timer 0 or use the Timer0Compare interrupt in your sketch.

## Example sketches provided

Example sketches:

- **Ibus2PWM**: converts iBUS data to Servo PWM format: Reads data from first servo channel and translates this to PWM signal to send to a normal servo
- **Ibus_singlemonitor**: monitor/debugger for receivers with a single iBUS pin (providing both servo and sensor data such as the Flysky X6B and Flysky FS-iA8X). Prints out servo channels to the standard serial output (PC debug window) and simulates 2 sensors with random values sent back to transmitter (as long as your receiver supports this). Requires Arduino board with 2 or more hardware serial ports (such as MEGA 2560)
- **Ibus_multimonitor**: monitor/debugger for receivers with a two separate iBUS pins (one for servo data and one for sensor data, such as the TGY-IA6B). Prints out servo channels to the standard serial output (PC debug window) and simulates 2 sensors with random values sent back to transmitter. Requires Arduino board with 3 or more hardware serial ports (such as MEGA 2560)
