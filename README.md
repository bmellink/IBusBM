# Arduino RC IBus protocol handler
Arduino library for RC IBUS protocol - servo (receive) and sensors/telemetry (send) using hardware UART.

The IBUS protocol is a half-duplex protocol developed by Flysky to control multiple servos and motors using a single digital line.
The protocol can also connect sensors to send back telemetry information to a RC transceiver. Currently the protocol only supports 3 sensor types: Voltage, Temperature and Motor speed (RPM). You can define up to 10 sensors using this library.

This library was written and tested for the TGY-IA6B receiver and should work for other receivers too.
The TGY-IA6B has 2 ibus pins: one for the servos (only output) and one for the sensors/telemetry
(which uses a half-duplex protocol to switch between output and input to poll for sensor data).

## Getting Started

To install this library use the **Clone or download > Download ZIP** button on the repository home page and then install the library in your Arduino environment using **Sketch > Include Library > Add .ZIP Library...**

This library is for AVR based Arduino boards only (Arduino MEGA, UNO, Nano, Micro, etc.).

### Prerequisites

The IBUS library requires a dedicated hardware serial (UART) port on the Arduino board. If your board only has one UART port you can still use that port for serial debug communication with your PC as long as you plan to use servo output mode only (the baud rate will be fixed at 115200 baud and you should only attach the UART TX pin to the USB-Serial converter). The ATMEGA boards typically have more than one UART port.

You have three options:
- If you plan to **only use servo output** IBUS data, you only need to connect the IBUS servo output pin from your receiver to the RX pin of the allocated UART. The TX pin of the UART will not be used by the library as there is no information sent back to the RC receiver.
- If you plan to **only use sensors** IBUS data, you will need to connect the IBUS sensor pin to both the RX and TX pin of the allocated UART. You need to include a diode (such as 1N4148) between the Arduino TX pin and the wire between the IBUS pin and the Arduino RX pin (cathode/solid ring of diode connected at Arduino TX pin) to handle the half-duplex protocol over a single wire.
- If you plan to **use both servo output and sensors**, your can use two different UART ports on your Arduino (see above) or use a single UART with a specific hardware circuit to tie the ports together. Please refer to the file xxxxxxxxx for the circuit design.

The main function to call for each IBUS instance is the IBus.loop() function. This function will process all characters that have been received by the serial interface and stored in the interteal buffer allocated by the standard hardware serial interface. For servo output you can call this function from within your main program loop() if you only need new servo data. Sensor communication is more time sensitive and the IBus.loop() function needs to be called at least once ever millisecond to guarantee no data is lost (see (https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry) for more details). The second example below shows how to use timer0 to realize this.

### Example code for servo output only

This example is for the MEGA 2560 board

```
#include <IBusBM.h>

IBusBM IBus;

void setup() {
  // initialize serial port for debug
  Serial.begin(115200);

  // IBUS connected to RX of serial1 port
  IBus.begin(Serial1);
  
  Serial.println("Start IBUS");
}

void loop() {
  // show first 8 servo channels
  for (int i=0; i<8 ; i++) {
    Serial.print(IBus.readChannel(i), HEX);
    Serial.print(" ");
  }
  Serial.print("Cnt=");
  Serial.println(IBus.cnt_rec); // count of how many times servo values have been updated
  delay(500);
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

  // IBUS servo connected to RX of serial1 port
  IBusServo.begin(Serial1);

  // IBUS sensor connected to RX + TX of serial2 port (1N4148 diode included in TX line - cathode connected to TX)
  IBusSensor.begin(Serial2);
  
  Serial.println("Start IBUS");

  // adding 2 sensors to generate some dummy data
  IBus.addSensor(IBUSS_RPM);
  IBus.addSensor(IBUSS_TEMP);
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
- uint16_t readChannel(uint8_t channelNr); // read the value of servo channel 0..9
- uint8_t addSensor(uint8_t type); // defines new sensor of type type, returns sensor number (first is number 1)
- void setSensorMeasurement(uint8_t adr, uint16_t value); // set value of sensor number adr to value (first sensor is numer 1)

The IBusBM class exposes the following counters. Counters are 1 byte (value 0..255) and values can be used by your code to understand if new data is available. 

- uint8_t cnt_rec; // count received number of servo messages
- uint8_t cnt_poll; // count received number of sensor poll messages
- uint8_t cnt_sensor; // count times a sensor value has been sent back

Counters can also be used to debug the hardware connections between the receiver and the Arduino board: If at least one sensor is defined, the RX pin will receive sensor poll messages (incrementing cnt_poll approximately evenry 7ms). Only if the TX pin is correctly connected to the receiver, the cnt_sensor counter will increment at the same rate as cnt_poll.

## Sensor types

The following sensor types are defined in IBusBM.h:

```
#define IBUSS_INTV 0 // Internal voltage (in 0.01)
#define IBUSS_TEMP 1 // Temperature (in 0.1 degrees, where 0=-40'C)
#define IBUSS_RPM  2 // RPM
#define IBUSS_EXTV 3 // External voltage (in 0.01
```

## Examples

Example sketches:

- Ibus2PWM: converts ibus data to Servo PWM format: Reads data from first servo channel and translates this to PWM signal to send to a traditional servo
- Ibus_monitor: print out servo channels to the standard serial output (PC debug window) and simulate 2 sensors with random values sent back to transmitter. Requires Arduino board with 2 or more hardware serial ports (such as MEGA 2560)
