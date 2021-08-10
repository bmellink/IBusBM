/*
 * Interface to the RC IBus protocol
 * 
 * Based on original work from: https://gitlab.com/timwilkinson/FlySkyIBus
 * Extended to also handle sensors/telemetry data to be sent back to the transmitter,
 * interrupts driven and other features.
 *
 * This lib requires a hardware UART for communication
 * Another version using software serial is here https://github.com/Hrastovc/iBUStelemetry
 * 
 * Explaination of sensor/ telemetry prtocol here: 
 * https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either  
 * version 2.1 of the License, or (at your option) any later version.
 *   
 * Created 12 March 2019 Bart Mellink
 * Updated 4 April 2019 to support ESP32
 * updated 13 jun 2019 to support STM32 (pauluzs)
 * Updated 21 Jul 2020 to support MBED (David Peverley)
 * Updated 10 Aug 2021 to support SAMD (Neil Hutchison)
 */

#include <Arduino.h>
#include "IBusBM.h"

// pointer to the first class instance to be used to call the loop() method from timer interrupt
// will be initiated by class constructor, then daisy channed to other class instances if we have more than one
IBusBM* IBusBMfirst = NULL;


// Interrupt on timer0 - called every 1 ms
// we call the IBusSensor.loop() here, so we are certain we respond to sensor requests in a timely matter
#ifdef ARDUINO_ARCH_AVR
SIGNAL(TIMER0_COMPA_vect) {
  if (IBusBMfirst) IBusBMfirst->loop();  // gets new servo values if available and process any sensor data
}
#elif defined _VARIANT_ARDUINO_STM32_
void  onTimer(stimer_t *htim) {
  if (IBusBMfirst) IBusBMfirst->loop();  // gets new servo values if available and process any sensor data
}
#else
void  onTimer() {
  if (IBusBMfirst) IBusBMfirst->loop();  // gets new servo values if available and process any sensor data
}
#endif


#if defined(ARDUINO_ARCH_MBED)
extern "C" {
  void TIMER4_IRQHandler_v() {
    if (NRF_TIMER4->EVENTS_COMPARE[0] == 1) {   
        onTimer();
        NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    }
  }
}
#endif

#if defined(__SAMD21G18A__)
void TC5_Handler (void) {
  onTimer();
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
}
#endif

/*
 * AFHDS 2A 
 * supports max 14 channels in this lib (with messagelength of 0x20 there is room for 14 channels)

  Example set of bytes coming over the iBUS line for setting servos: 
    20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
  Explanation
    Protocol length: 20
    Command code: 40 
    Channel 0: DB 5  -> value 0x5DB
    Channel 1: DC 5  -> value 0x5Dc
    Channel 2: 54 5  -> value 0x554
    Channel 3: DC 5  -> value 0x5DC
    Channel 4: E8 3  -> value 0x3E8
    Channel 5: D0 7  -> value 0x7D0
    Channel 6: D2 5  -> value 0x5D2
    Channel 7: E8 3  -> value 0x3E8
    Channel 8: DC 5  -> value 0x5DC
    Channel 9: DC 5  -> value 0x5DC
    Channel 10: DC 5 -> value 0x5DC
    Channel 11: DC 5 -> value 0x5DC
    Channel 12: DC 5 -> value 0x5DC
    Channel 13: DC 5 -> value 0x5DC
    Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
 */
 
 /*
  *  AFHDS 3
  *  supports max 18 channels in this lib 
     (with messagelength of 0x20 there is room for 18 channels, since channels 14-18 are interleaved)

  Example set of bytes coming over the iBUS line for setting servos: 
    20 40 DB C5 DC B5 54 55 DC C5 E8 B3 D0 57 D2 C5 E8 B3 DC 55 DC C5 DC B5 DC 55 DC 5 DC 5 DA EC
  Explanation
    Protocol length: 20
    Command code: 40 
    Channel 0: DB c5  -> value 0x5DB (low 14)
    Channel 1: DC b5  -> value 0x5Dc (mid 14)
    Channel 2: 54 55  -> value 0x554 (high 14)
    Channel 3: DC c5  -> value 0x5DC (low 15)
    Channel 4: E8 b3  -> value 0x3E8 (mid 15)
    Channel 5: D0 57  -> value 0x7D0 (high 15)
    Channel 6: D2 c5  -> value 0x5D2 (low 16)
    Channel 7: E8 b3  -> value 0x3E8 (mid 16)
    Channel 8: DC 55  -> value 0x5DC (high 16)
    Channel 9: DC c5  -> value 0x5DC (low 17)
    Channel 10: DC b5 -> value 0x5DC (mid 17)
    Channel 11: DC 55 -> value 0x5DC (high 17)
    Channel 12: DC 05 -> value 0x5DC
    Channel 13: DC 05 -> value 0x5DC
    Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
 */

#if defined(_VARIANT_ARDUINO_STM32_)
void IBusBM::begin(HardwareSerial &serial, TIM_TypeDef * timerid, int8_t rxPin, int8_t txPin) {
#else
void IBusBM::begin(HardwareSerial &serial, int8_t timerid, int8_t rxPin, int8_t txPin) {
#endif

  #ifdef ARDUINO_ARCH_ESP32
    serial.begin(115200, SERIAL_8N1, rxPin, txPin);
  #else
    serial.begin(115200, SERIAL_8N1);
  #endif

  this->stream = &serial;
  this->state = DISCARD;
  this->last = millis();
  this->ptr = 0;
  this->len = 0;
  this->chksum = 0;
  this->lchksum = 0;

  // we need to process the iBUS sensor protocol handler frequently enough (at least once each ms) to ensure the response data
  // from the sensor is sent on time to the receiver
  // if timerid==IBUSBM_NOTIMER the user is responsible for calling the loop function
  this->IBusBMnext = IBusBMfirst;

  if (!IBusBMfirst && timerid != IBUSBM_NOTIMER) {
    #ifdef ARDUINO_ARCH_AVR
      // on AVR architectures Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the TIMER0_COMPA_vect interrupt
      OCR0A = 0xAF;
      TIMSK0 |= _BV(OCIE0A);
    #else
      // on other architectures we need to use a time
      #if defined(ARDUINO_ARCH_ESP32) 
        hw_timer_t * timer = NULL;
        timer = timerBegin(timerid, F_CPU / 1000000L, true); // defaults to timer_id = 0; divider=80 (1 ms); countUp = true;
        timerAttachInterrupt(timer, &onTimer, true); // edge = true
        timerAlarmWrite(timer, 1000, true);  //1 ms
        timerAlarmEnable(timer);
      #elif defined(_VARIANT_ARDUINO_STM32_)
        // see https://github.com/stm32duino/wiki/wiki/HardwareTimer-library
        HardwareTimer *stimer_t = new HardwareTimer(timerid);
        stimer_t->setOverflow(1000, HERTZ_FORMAT); // 1000 Hz
        stimer_t->attachInterrupt(onTimer);
        stimer_t->resume();
      #elif defined(ARDUINO_ARCH_MBED)
        NRF_TIMER4->TASKS_STOP = 1; // Stop timer
        NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Counter Mode
        NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
        NRF_TIMER2->TASKS_CLEAR = 1;               // clear the task first to be usable for later

        // Set prescaler & compare register.
        // Prescaler = 0 gives 16MHz timer. 
        // Prescaler = 4 (2^4) gives 1MHz timer. 
        NRF_TIMER4->PRESCALER = 4 << TIMER_PRESCALER_PRESCALER_Pos;  
        NRF_TIMER4->CC[0] = 1000; 
  
        // Enable interrupt on Timer 4 for CC[0] compare match events
        NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
        NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
 
        NVIC_EnableIRQ(TIMER4_IRQn);

        NRF_TIMER4->TASKS_START = 1;      // Start TIMER2
      #elif defined(__SAMD21G18A__)
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;  // TC Reset
	while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until TC5 is done syncing
	while (TC5->COUNT16.CTRLA.bit.SWRST);
	
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16; // Set Timer counter 5 Mode to 16 bits, it will become a 16bit counter ('mode1' in the datasheet)
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC5 waveform generation mode to 'match frequency'
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE; //set prescaler
	TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / 1000);  // //set the compare-capture register to 1ms
	while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);  //wait until TC5 is done syncing
	
	// Configure interrupt request
	NVIC_DisableIRQ(TC5_IRQn);
	NVIC_ClearPendingIRQ(TC5_IRQn);
	NVIC_SetPriority(TC5_IRQn, 0);
	NVIC_EnableIRQ(TC5_IRQn);
       
	TC5->COUNT16.INTENSET.bit.MC0 = 1; // Enable the TC5 interrupt request
	while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until TC5 is done syncing
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; // Start counter
	while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //wait until snyc'd
      #else
        // It should not be too difficult to support additional architectures as most have timer functions, but I only tested AVR and ESP32
        #warning "Timing only supportted for AVR, ESP32, STM32 and SAMD21 architectures. Use timerid IBUSBM_NOTIMER"
      #endif
    #endif
  }
  IBusBMfirst = this; 
}

// called from timer interrupt or mannually by user (if IBUSBM_NOTIMER set in begin())
void IBusBM::loop(void) {

  // if we have multiple instances of IBusBM, we (recursively) call the other instances loop() function
  if (IBusBMnext) IBusBMnext->loop(); 

  // only process data already in our UART receive buffer 
  while (stream->available() > 0) {
    // only consider a new data package if we have not heard anything for >3ms
    uint32_t now = millis();
    if (now - last >= PROTOCOL_TIMEGAP){
      state = GET_LENGTH;
    }
    last = now;
    
    uint8_t v = stream->read();
    switch (state) {
      case GET_LENGTH:
        if (v <= PROTOCOL_LENGTH && v > PROTOCOL_OVERHEAD) {
          ptr = 0;
          len = v - PROTOCOL_OVERHEAD;
          chksum = 0xFFFF - v;
          state = GET_DATA;
        } else {
          state = DISCARD;
        }
        break;

      case GET_DATA:
        buffer[ptr++] = v;
        chksum -= v;
        if (ptr == len) {
          state = GET_CHKSUML;
        }
        break;
        
      case GET_CHKSUML:
        lchksum = v;
        state = GET_CHKSUMH;
        break;

      case GET_CHKSUMH:
        // Validate checksum
        if (chksum == (v << 8) + lchksum) {
          // Checksum is all fine Execute command - 
          uint8_t adr = buffer[0] & 0x0f;
          if (buffer[0]==PROTOCOL_COMMAND40) {
	    uint8_t pos = 1;
	    uint8_t chan = 0;
	    uint8_t chan_high = 14;
            // Valid servo command received - extract channel data
	    //for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2) {
	    //  channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
	    //}
	    //cnt_rec++;
	    
	    // This code is updated to work for AFHDS 3 ... but it also works for 2A, so we can replace the old code.
	    // Given the interleaving it's a little more work
	    for (int y=0; y<4; y++) {
	      // From here on, we're looking at servo channel data ...
	      // First channel // chan 15 byte 1
	      channel[chan] = buffer[pos] | ((buffer[pos + 1] & 0x0f) << 8);
	      pos++;
	      chan++;
	      channel[chan_high] = (buffer[pos] >> 4);
	      pos++;
	      // Second channel // chan 15 byte 2
	      channel[chan] = buffer[pos] | ((buffer[pos + 1] & 0x0f) << 8);
	      pos++;
	      chan++;
	      channel[chan_high] = channel[chan_high] | ((buffer[pos] >> 4) << 4);
	      pos++;
	      // Third channel // chan 15 byte 3
	      channel[chan] = buffer[pos] | ((buffer[pos + 1] & 0x0f) << 8);
	      pos++;
	      chan++;
	      channel[chan_high] = channel[chan_high] | ((buffer[pos] >> 4) << 8);
	      chan_high++;
	      pos++;
	    }
	    // Ok, we should be up to channel 12 /18 ... only 2 channels left.
	    // Channels 12/13 are the last couple of bytes.
	    for (int z=0; z<2; z++){
	      channel[chan] = buffer[pos] | ((buffer[pos + 1] & 0x0f) << 8);
	      pos+=2;
	      chan++;
	    }
          } else if (adr<=NumberSensors && adr>0 && len==1) {

            // all sensor data commands go here
            // we only process the len==1 commands (=message length is 4 bytes incl overhead) to prevent the case the
            // return messages from the UART TX port loop back to the RX port and are processed again. This is extra
            // precaution as it will also be prevented by the PROTOCOL_TIMEGAP required
           sensorinfo *s = &sensors[adr-1];
           delayMicroseconds(100);
            switch (buffer[0] & 0x0f0) {
              case PROTOCOL_COMMAND_DISCOVER: // 0x80, discover sensor
                cnt_poll++;  
                // echo discover command: 0x04, 0x81, 0x7A, 0xFF 
                stream->write(0x04);
                stream->write(PROTOCOL_COMMAND_DISCOVER + adr);
                chksum = 0xFFFF - (0x04 + PROTOCOL_COMMAND_DISCOVER + adr);
                break;
              case PROTOCOL_COMMAND_TYPE: // 0x90, send sensor type
                // echo sensortype command: 0x06 0x91 0x00 0x02 0x66 0xFF 
                stream->write(0x06);
                stream->write(PROTOCOL_COMMAND_TYPE + adr);
                stream->write(s->sensorType);
                stream->write(s->sensorLength);
                chksum = 0xFFFF - (0x06 + PROTOCOL_COMMAND_TYPE + adr + s->sensorType + s->sensorLength);
                break;
              case PROTOCOL_COMMAND_VALUE: // 0xA0, send sensor data
                cnt_sensor++;
                uint8_t t;
                // echo sensor value command: 0x06 0x91 0x00 0x02 0x66 0xFF 
                stream->write(t = 0x04 + s->sensorLength);
                chksum = 0xFFFF - t;
                stream->write(t = PROTOCOL_COMMAND_VALUE + adr);
                chksum -= t;
                stream->write(t = s->sensorValue & 0x0ff);
                chksum -= t;
                stream->write(t = (s->sensorValue >> 8) & 0x0ff); 
                chksum -= t;
                if (s->sensorLength==4) {
                  stream->write(t = (s->sensorValue >> 16) & 0x0ff); 
                  chksum -= t;
                  stream->write(t = (s->sensorValue >> 24) & 0x0ff); 
                  chksum -= t;                  
                }
                break;
              default:
                adr=0; // unknown command, prevent sending chksum
                break;
            }
            if (adr>0) {
              stream->write(chksum & 0x0ff);
              stream->write(chksum >> 8);              
            }
          }
        }
        state = DISCARD;
        break;

      case DISCARD:
      default:
        break;
    }
  }
}

uint16_t IBusBM::readChannel(uint8_t channelNr) {
  if (channelNr < PROTOCOL_CHANNELS) {
    return channel[channelNr];
  } else {
    return 0;
  }
}

uint8_t IBusBM::addSensor(uint8_t type, uint8_t len) {
  // add a sensor, return sensor number
  if (len!=2 && len!=4) len = 2;
  if (NumberSensors < SENSORMAX) {
    sensorinfo *s = &sensors[NumberSensors];
    s->sensorType = type;
    s->sensorLength = len;
    s->sensorValue = 0;
    NumberSensors++;
  }
  return NumberSensors;
}

void IBusBM::setSensorMeasurement(uint8_t adr, int32_t value) {
   if (adr<=NumberSensors && adr>0)
     sensors[adr-1].sensorValue = value;
}

