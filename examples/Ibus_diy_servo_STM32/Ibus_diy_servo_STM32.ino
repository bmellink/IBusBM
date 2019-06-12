/*
  Ibus_diy_servo_STM32 example

  For (large) diy DC servo's, like windshield wiper motors.
  Change the  P I D settings for servo response and tuning.
  USB Serial speed: 115200 (to display debug information on your PC).

  Connect Ibus reciever to STM32 Serial1 (PA10 for STM32F103C8T6/blue pill)
  Connect motor shaft feedback potentiometer to GND, 3,3V and the wiper to A0
  For use with H-bridge based on BTS7960, like IBT-2 modules:
    Connect GND to GND
    Connect L-EN and R-EN to LED_BUILTIN pin (PC13 for STM32F103C8T6/blue pill)
    Connect LPWM to LPWM_pin and LPWM to RPWM_pin(PB6 and PB7 for STM32F103C8T6/blue pill)

  !!! NOTE: The IBUS.cnt_rec is used to check if the reciever got disconnected or stopped sending valid Ibus messages:
  !!!   This can only be used as FAILSAFE on non Telemetry Recievers (Like the flysky a8s v2, which stops sending Ibus messages on transmitter loss)
  !!!   For all Telemetry Recievers set up FAILSAFE on ARMED Channel (Like the flysky x6b which keep sending Ibus messages even on transmitter loss)

*/
#include <IBusBM.h>                                                                 // https://github.com/bmellink/IBusBM
#include <PID_v1.h>                                                                 // https://github.com/br3ttb/Arduino-PID-Library/

byte ARMED_chan = 6;                                                                // Ibus channel for Arm switch
byte STEER_chan = 1;                                                                // Ibus channel for Steering input

byte LPWM_pin = PB6;                                                                // Drive Left  PWM output pin
byte RPWM_pin = PB7;                                                                // Drive Right PWM output pin
byte ANGLE_pin = A0;                                                                // Potentiometer input pin, servo angle feedback
byte ARMED_pin = LED_BUILTIN;                                                       // Armed/Enable output pin, enables drive

byte MIN_pwm = 15;                                                                  // Min pwm needed to start motor (0 < MAX_pwm)
byte MAX_pwm = 220;                                                                 // Max pwm allowed to run motor (MIN_pwm < 255)

double P = 2;                                                                       // Servo P setting
double I = 0;                                                                       // Servo I setting
double D = 0;                                                                       // Servo D setting

byte IbusCount = 0;                                                                 // Ibus message counter
char MillisCount = 0 ;                                                              // Millisecond counter
char IbusInterval =  8 ;                                                            // Interval between Ibus messages in Milliseconds
unsigned long CurrentMillis = 0;                                                    // CurrentMillis variable

double Setpoint, Input, Output;                                                     // PID variables
PID SERVO(&Input, &Output, &Setpoint, P, I , D, DIRECT);                            // Initialize servo pid

IBusBM IBUS;                                                                        // initialize IbusMonitor

void setup() {                                                                      // Put your setup code here, to run once:

  pinMode(ANGLE_pin, INPUT);                                                        // Configure Pin Modes
  pinMode(ARMED_pin, OUTPUT);
  pinMode(LPWM_pin, OUTPUT);
  pinMode(RPWM_pin, OUTPUT);

  digitalWrite(ARMED_pin, LOW);                                                     // Set intitial Pin states
  digitalWrite(LPWM_pin, LOW);
  digitalWrite(RPWM_pin, LOW);

  SERVO.SetMode(AUTOMATIC);                                                         // SERVO PID Setup
  SERVO.SetOutputLimits(-511, 511);                                                 // Limit to +/- half of analogRead resolution (0-1023)
  SERVO.SetSampleTime(IbusInterval);                                                // Sample at IbusInterval (8ms, 125Hz)

  Serial.begin(115200);                                                             // Start Serial usb connection

  IBUS.begin(Serial1, IBUSBM_NOTIMER);                                              // Start Ibus connection on serial1 without timer, call IBUS.loop() in Millisecond loop.

  while (IBUS.cnt_rec == 0) {                                                       // Wait until first Ibus messages is recieved
    IBUS.loop();                                                                    // Call the Ibus loop to check for Ibus messages from the receiver
    if (Serial.available() ) {                                                      // Only show debug message if usb is connected
      Serial.println("Starting iBUS Servo, Waiting for Reciever");
    }
    delay(1);                                                                       // Wait 1 Milllisecond before checking for Ibus messages again
  }

}

void loop() {                                                                       // Put your main code here, to run repeatedly:

  if (CurrentMillis !=  millis()) {                                                 // Create a 1 KHZ Millisecond loop, compare CurrentMillis counter to the Actual Milliseconds

    IBUS.loop();                                                                    // Call the Ibus loop to check for Ibus messages from the receiver
    MillisCount++;                                                                  // Increment the Millisecond counter
    CurrentMillis = millis();                                                       // Update the Current Millisecond counter to the Actual Milliseconds

    if (MillisCount > IbusInterval - 1 ) {                                          // Create a 125 HZ IbusInterval loop

      bool ARMED = map(IBUS.readChannel(ARMED_chan - 1), 1000 , 2000 , 0 , 1 );     // Read and convert ARMED_chan
      int STEER = (IBUS.readChannel(STEER_chan - 1));                               // Read ibus steer input from STEER_chan
      int ANGLE = analogRead(ANGLE_pin);                                            // Read Angle from potentiometer

      Setpoint = map(STEER, 1000, 2000, -511, 511);                                 // Convert STEER Value to match PID limits
      Input = map(ANGLE, 0, 1023, -511, 511);                                       // Convert ANGLE Value to match feedback PID limits
      SERVO.Compute();                                                              // Calculate PID output

      if (IBUS.cnt_rec == IbusCount) {                                              // Check if IBUS.cnt_rec has updated, disable when no New Ibus messages.
        digitalWrite(ARMED_pin, LOW);
        analogWrite(LPWM_pin, 0);
        analogWrite(RPWM_pin, 0);
        if (Serial.available() ) {                                                  // Only show debug message if usb is connected
          Serial.println("Waiting for Reciever");
        }
      }

      else {                                                                        // New Ibus message recieved

        if (Serial.available() ) {                                                  // Only show debug message if usb is connected
          for (int i = 0; i < 10 ; i++) {                                           // Show first 10 ibus channels
            Serial.print(IBUS.readChannel(i));
            Serial.print(" ");
          }
          Serial.print("Cnt=");
          Serial.print(IBUS.cnt_rec);                                               // Show Ibus message counter
          Serial.print (" Armed ");
          Serial.print (ARMED);                                                     // Show ARMED State
          Serial.print (" , Angle ");
          Serial.print (ANGLE);                                                     // Show ANGLE from Potentiometer
          Serial.print (" , PWM ");
          Serial.println( int(constrain(Output / 2, -MAX_pwm, MAX_pwm))*ARMED );    // Convert and Show PWM Value
        }

        digitalWrite(ARMED_pin, ARMED);                                             // Write enable pin according to ARMED state

        if (Output > 0) {                                                           // Drive left
          Output = int(constrain(Output / 2, MIN_pwm, MAX_pwm)) * ARMED;            // Convert output to analogWrite resolution with PWM limits
          analogWrite(RPWM_pin, 0);                                                 // Disable RPWM_pin
          analogWrite(LPWM_pin, Output);                                            // Write PWM on LPWM_pin
        }
        else {                                                                      // Drive right
          Output = abs(int(constrain(Output / 2, -MAX_pwm, -MIN_pwm))) * ARMED;;    // Convert output to analogWrite resolution with PWM limits
          analogWrite(LPWM_pin, 0);                                                 // Disable LPWM_pin
          analogWrite(RPWM_pin, Output);                                            // Write PWM on RPWM_pin
        }

      }

      IbusCount = IBUS.cnt_rec;                                                     // Update Ibus message counter
      MillisCount = 0;                                                              // Reset Millisecond counter
    }
  }
}
