/*
  Monitor IBUS signals and show output on the serial monitor
  
  Requires Arduino board with multiple UARTs (such as ATMEGA 2560)
  - serial0 - monitor output
  - serial1 - connected to the ibus

  Sensors do not seem to work on TGY-iA6C, 
  
  On the TGY-IA6B there are 2 ibus pins: one for the servos (only output) and one for the sensors/telemetry
  (which uses a half duplex protocol to switch between output and input to poll the sensors)

  You have 4 options:
  - If you connect the serial1 (RX1) pin to the ibus servo pin you will only see the servo values on screen
  - If you connect the serial1 (RX1) pin to the ibus sensor/telemetry pin, you will only see the polling for sensors
    (the variable IBus.sensor1 will increment every 7ms, but the data will not be sent back to your transmitter)
  - If you connect both the ibus servo and sensor pin to the RX1 pin of the serial1 using a diode and resistor as described in
    https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry you will see both servo data and
    polling for sensor data. However the sensor data is still not sent back to the transmitter
  - If you add another diode between the TX1 pin of the Arduino and the telemetry ibus pin of the receiver (cathode=white ring
    of the diode at the side of TX1) you will also see the sensor data being send back to the receiver (S2 and S3 counters 
    also increase in value)

  Alternatively you can also use two different UARTS for servo and telemetry data by defining two different IBus objects:
    IBusBM IBusServo;
    IBusBM IBusSensor;
  assigning them to the right Serial1/Serial2 ports in setup() and updating values in SIGNAL timer interrupt
  Strictly speaking there is no need to poll IbusServo.loop() in the timer interrupt routine. Interrupt driven polling is
  only needed for IbusSensor. Polling of the Servo can also be done in the main loop() function
  
  sensor types defined in IBusBM.h:
  
  #define IBUSS_INTV 0 // Internal voltage (in 0.01)
  #define IBUSS_TEMP 1 // Temperature (in 0.1 degrees, where 0=-40'C)
  #define IBUSS_RPM  2 // RPM
  #define IBUSS_EXTV 3 // External voltage (in 0.01

*/

#include "IBusBM.h"
IBusBM IBus; // should actually be volatile as this may be interrupt driven

void setup() {
  // initialize serial port for monitor
  Serial.begin(115200,SERIAL_8N1);

  // IBUS connected to serial1
  IBus.begin(Serial1);
  
  Serial.println("Start IBUS monitor");

  // adding 2 sensors
  IBus.addSensor(IBUSS_RPM);
  IBus.addSensor(IBUSS_TEMP);

  // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the TIMER0_COMPA_vect interrupt
  // we need to process the IBUS protocol handler here to ensure it always runs on time and we reply back sensor data to the receiver
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}


#define TEMPBASE 400    // base value for 0'C

// sensor values
int speed=0;
int temp=TEMPBASE+20; // start at 20'C

void loop() {
  // show first 8 servo channels
  for (int i=0; i<8 ; i++) {
    Serial.print(IBus.readChannel(i), HEX);
    Serial.print(" ");
  }
  Serial.print("Cnt=");
  Serial.print(IBus.cnt_rec); // count of how many times servo values have been updated
  Serial.print(" S1=");
  Serial.print(IBus.sensor1); // count of polling for sensor existance
  Serial.print(" S2=");
  Serial.print(IBus.sensor2); // count of polling for sensor type
  Serial.print(" S3=");
  Serial.println(IBus.sensor3); // count of polling for sensor value
  
  IBus.setSensorMeasurement(1,speed);
  speed +=20;                           // increase motor speed by 10 RPM
  IBus.setSensorMeasurement(2,temp++); // increase temperature by 0.1 'C every loop
  delay(500);
}

// Interrupt on timer0 - called as part of timer0 - which is running at 1ms intervals (internally used for millis()
// we call the IBus.loop() here, so we are certain we respond on sensor request in a timely matter
SIGNAL(TIMER0_COMPA_vect) {
  interrupts(); // allow interrupts to run as the IBus loop may take max 100 microseconds to run
  IBus.loop();  // gets new servo values if available and process any sensor data
}



/*
 * 

Code als je kaal de bytes wilt laten zien op scherm

void setup() {
  // initialize both serial ports:
  Serial.begin(115200,SERIAL_8N1);
  Serial1.begin(115200,SERIAL_8N1);
  Serial.println("Start IBUS monitor");
}

int Byte1=0, Byte2=0;

void loop() {
  
  // read from port 1, send to port 0:
  if (Serial1.available()) {
    int Byte1 = Serial1.read();
    Serial.print(Byte1, HEX); Serial.print(" ");
    if (Byte1==0x040 && Byte2==0x020) Serial.println();
    Byte2 = Byte1;
    
  }

 }

 */
