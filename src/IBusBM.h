/*
 * Interface to the RC IBus protocol
 * 
 * Based on original work from: https://gitlab.com/timwilkinson/FlySkyIBus
 * Extended to also handle sensors/telemetry data to be sent back to the transmitter
 * This lib requires a hardware UART for communication
 * Another version using software serial is here https://github.com/Hrastovc/iBUStelemetry)
 * 
 * Explaination of sensor/ telemetry prtocol and the circuit (R + diode) to combine the two pins here: 
 * https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either  
 * version 2.1 of the License, or (at your option) any later version.
 *   
 * Created 12 March 2019 Bart Mellink
 */
#ifndef IBusBM_h
#define IBusBM_h

#include <inttypes.h>

#define IBUSS_INTV 0 // Internal voltage (in 0.01)
#define IBUSS_TEMP 1 // Temperature (in 0.1 degrees, where 0=-40'C)
#define IBUSS_RPM  2 // RPM
#define IBUSS_EXTV 3 // External voltage (in 0.01

class HardwareSerial;
class Stream;

class IBusBM {
public:
  void begin(HardwareSerial& serial);
  void begin(Stream& stream);
  void loop(void); // used internally
  uint16_t readChannel(uint8_t channelNr); // read servo channel 0..9
  uint8_t addSensor(uint8_t type); // usually 2, returns address
  void setSensorMeasurement(uint8_t adr, uint16_t value);
  
  uint8_t cnt_poll; // count received number of sensor poll messages
  uint8_t cnt_sensor; // count times a sensor value has been sent back
  uint8_t cnt_rec; // count received number of servo messages
  
private:
  enum State {GET_LENGTH, GET_DATA, GET_CHKSUML, GET_CHKSUMH, DISCARD};

  static const uint8_t PROTOCOL_LENGTH = 0x20;
  static const uint8_t PROTOCOL_OVERHEAD = 3; // packet is <len><cmd><data....><chkl><chkh>, overhead=cmd+chk bytes
  static const uint8_t PROTOCOL_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
  static const uint8_t PROTOCOL_CHANNELS = 14;
  static const uint8_t PROTOCOL_COMMAND40 = 0x40;        // Command to set servo or motor speed is always 0x40
  static const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80; // Command discover sensor (lowest 4 bits are sensor)
  static const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;     // Command discover sensor (lowest 4 bits are sensor)
  static const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;    // Command send sensor data (lowest 4 bits are sensor)
  static const uint8_t SENSORMAX = 10; // Max number of sensors
  
  uint8_t state;
  Stream* stream;
  uint32_t last;
  uint8_t buffer[PROTOCOL_LENGTH];
  uint8_t ptr;
  uint8_t len;
  uint16_t channel[PROTOCOL_CHANNELS];
  uint16_t chksum;
  uint8_t lchksum;
  uint8_t sensorType[SENSORMAX]; 
  uint16_t sensorValue[SENSORMAX]; 
  uint8_t NumberSensors;
  // pointer to the next class instance to be used to call the loop() method from timer interrupt
  IBusBM* IBusBMnext = NULL;
};


#endif
