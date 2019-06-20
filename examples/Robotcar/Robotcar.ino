/*
 * Robotcar.ino
 *
 * Example to create a remote controlled car where motors on individual left and right wheels are used
 * for both driving and steering. The code uses the following two libraries:
 * VNH3SP30 motor driver library - https://github.com/bmellink/VNH3SP30
 * IBusBM to handle the IBus signals from Flysky/Turnigi RC controller - https://github.com/bmellink/IBusBM
 *
 * This code was tested with a 6 wheel RC car where all wheels have their own motor. All 3 wheels on the left 
 * hand side are connected in parallel to a VNH3ASP30 motor controller which is wired to pins 2,3,4 of the Arduino.
 * All 3 wheels on the right hand side are connected in parallel to a second CNH30ASP30 motor controller and
 * wired to pins 5,6,7 pf the Arduino.
 *
 * The physical contruction of the car requires the left hand motors to runs forward and the right hand set of
 * motors to run backwards to drive the car in the forward direction. This means the speed command for the left
 * motors should be in the range 0..+400 while the right motors get a 0..-400 segnal. As it is easier to use 
 * a range of 0..+400 for driving forward and 0..-400 for driving backward we need to swap direction for one set 
 * of motors. This can be handled:
 * - in hardware: by swapping red and black wires on the motors
 * - in hardware: by swapping the INA and INB pins when connecting the pins to the Arduino
 * - in software: by swapping the INA and INB pins when calling the begin() function
 *
 * Turning the car is implementing by providing different speeds to the right and left motors. When standing still
 * (speed==0) the car can turn in place by simply driving the left and right motors in oposite directions.
 *
 * This example uses the MEGA 2560, but any board with sufficient pins and async interfaces should work.
 *
 * The remote controller setup used in this example is a Turnigy FS-MT6 (transmitter) and TGY-IA6B (receiver). The IBUS
 * signals received from the various channels translate into values between 1000-2000 (midpoint = 1500).
 * Channel assignment (in my setup) is:
 *  channel 0 - right lever LR --> use to turn
 *  channel 1 - left lever UP/DOWN
 *  channel 2 - right lever UP/DOWN --> use for speed
 *  channel 3 - left lever LR
 *  channel 4 - switch back left (up=1000, mid=1500, down=2000)
 *  channel 5 - switch front (up=1000, down=2000) --> use to go forward/backwards
 *  channel 6 - potmeter
 *  channel 7 - switch back right (up=1000, mid=1500, down=2000)
 *
 * We use channels 0 (turn), 2 (speed) and 5 (switch forward/backwards) in this example. 
 * Note: in my case the right lever of the transmitter is designed for airplane usage and does not have a spring to force
 * a resting position in the middle. This is why I use channel 5 (switch) to change the direction of the car. If you have
 * another transmitter or want the right hand lever to also control the car direction, you can change the code below.
 *
 * The Turnigy receiver is connected to Serial1 of the MEGA 2560: pin 19 (RX) and 18 (TX)
 * 
 */

#include <VNH3SP30.h>
#include <IBusBM.h>

VNH3SP30 Motor1;    // define control object for motor 1 (left side)
VNH3SP30 Motor2;    // define control object for motor 2 (right side)

// motor pins (DIAG and CS pins not used in this example)
// In this example the RED and BLACK wires of the motors on the right side of the car are swapped
// to ensure these motors turn in the other direction, so there is no need to reverse INA/INB pins

#define M1_PWM 2    // pwm pin motor
#define M1_INA 3    // control pin INA
#define M1_INB 4    // control pin INB

#define M2_PWM 5    // pwm pin motor
#define M2_INA 6    // control pin INA
#define M2_INB 7    // control pin INB


IBusBM IBus; // IBus object for receivig signals from transmitter/receiver

void setup() {
  Serial.begin(115200);   

  // Setting up motor connections. 
  Motor1.begin(M1_PWM, M1_INA, M1_INB, -1, -1);    // Motor 1 object connected through specified pins 
  Motor2.begin(M2_PWM, M2_INA, M2_INB, -1, -1);    // Motor 2 object connected through specified pins 

  IBus.begin(Serial1);    // iBUS connected to Serial1: 19 (RX) and 18 (TX)

  // We have to wait for the receiver to receive data from the transmitter (transmitter needs to be turned on)
  // as the channel values all read 0 as long as the transmitter is turned off at boot time.
  // We do not want the car to drive full speed backwards out of control.
  Serial.println("Wait for receiver");
  while (IBus.cnt_rec==0) delay(100);
  Serial.println("Init done");
}

// braking not used in thix example
void brake(int brakePower) {
  Motor1.brake(brakePower);
  Motor2.brake(brakePower);
}

void speedturn(int speed, int angle) {
  // set speed (-400 -> +400) and turn (-400 -> +400)
  // turn vehicle by providing different speed settings to the motors.
  // angle can be positive (right turn) or negative (left turn).
  // If the vehicle is already stopped, the vehicle will turn in place.
  Motor1.setSpeed(speed + angle);
  Motor2.setSpeed(speed - angle);
}

int savespd=0, saveturn=0;

void loop() {
  int spd, turn;
  // speed depends on front switch (channel 5) (forward/backwards) and channel 2 (speed)
  spd = ((int) IBus.readChannel(2)-1050); 
  // every value below 1050 we interprete as stop 
  if (spd<0) spd=0; else spd = (spd*4)/9; // value could reach (2000-1050)*4/9 = 422, but setspeed() will max at 400
  if (IBus.readChannel(5)>1500) spd=-spd; // backward/forward depends on switch at channel 5
  
  // turn depends on channel 0, scale down to -200, +200
  turn = (((int) IBus.readChannel(0)-1500)*4)/10; 

  // set combined speed and turn (if speed==0, then only turn in place)
  speedturn(spd, turn);

  if (savespd != spd || saveturn != turn) {
    Serial.print("speed="); Serial.print(spd); // display speed
    Serial.print(" turn="); Serial.println(turn); // display turn 
    savespd = spd;
    saveturn = turn;
  }
  delay(100);
}
