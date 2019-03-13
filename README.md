# IBusBM
Arduino library for RC IBUS protocol - servo (receive) and sensors/telemetry (send) using hardware UART

The IBUS protocol is a half duplex protocol developed by flysky to control multiple servos and motors using a single digital line.
The protocol can also connect sensors to send back telemetry information to a RC transceiver. Currently the protocol only supports 3 sensor types: Voltage, Temperature and Motor speed (RPM). You can define up to 10 sensors using this library.

This library was written and tested for the TGY-IA6B receiver, but should work for other receivers too.
The TGY-IA6B has 2 ibus pins: one for the servos (only output) and one for the sensors/telemetry
(which uses a half duplex protocol to switch between output and input to poll the sensors)

## Getting Started

To install this library use the **Clone or download > Download ZIP** button on the repository home page and then install the library via **Sketch > Include Library > Add .ZIP Library...**

### Prerequisites

The IBUS library requires a dedicated hardware serial (UART) port on the Arduino board. If your board only has one free UART port you can still use that port for serial debug communication with your PC at 115200 baud if you plan to use servo output mode only. The ATMEGA boards typically have more than one UART port.

You have three options:
- If you plan to **only use servo output** IBUS data, you only need to connect the IBUS servo output pin from your receiver to the RX pin of the allocated UART. The TX pin of the UART will not be used by the library as there is no information sent back to the RC receiver.
- If you plan to **only use sensors** IBUS data, you will need to connect the IBUS sensor pin to both the RX and TX pin of the allocated UART. You need to include a diode (such as 1N4148) between the Arduino TX pin and the wire between the IBUS pin and the Arduino RX pin (kathody/solid ring of diode connected at Arduino TX pin) to handle the half-duplex protocol over a single wire.
- If you plan to **use both servo output and sensors**, your can use two different UART ports on your Arduino (see above) or use a single UART with a specific hardware circuit to tie the ports together. Please refer to the file xxxxxxxxx for the circuit design.

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
  IBus.loop();
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
  // initialize serial port for debug
  Serial.begin(115200);

  // IBUS servo connected to RX of serial1 port
  IBusServo.begin(Serial1);

  // IBUS sensor connected to RX + TX of serial2 port (diode added to TX line)
  IBusSensor.begin(Serial2);
  
  Serial.println("Start IBUS");

  // adding 2 sensors
  IBus.addSensor(IBUSS_RPM);
  IBus.addSensor(IBUSS_TEMP);

  // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the TIMER0_COMPA_vect interrupt
  // we need to process the IBUS sensor protocol handler here to ensure it always runs on time and we reply back sensor data to the receiver
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

void loop() {
  IBusServo.loop(); // read the latest servo values
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

// Interrupt on timer0 - called as part of timer0 - which is running at 1ms intervals (internally used for millis()
// we call the IBus.loop() here, so we are certain we respond on sensor request in a timely matter
SIGNAL(TIMER0_COMPA_vect) {
  IBusSensor.loop();  // gets new servo values if available and process any sensor data
}

```



## Sensor types

The following sensor types are defined in IBusBM.h:

```
#define IBUSS_INTV 0 // Internal voltage (in 0.01)
#define IBUSS_TEMP 1 // Temperature (in 0.1 degrees, where 0=-40'C)
#define IBUSS_RPM  2 // RPM
#define IBUSS_EXTV 3 // External voltage (in 0.01
```

