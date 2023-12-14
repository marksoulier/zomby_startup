// Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

// See limitations of Arduino SoftwareSerial
// Rx pin: 18
// TX pin: 19
// Pins other than 0 and 1 must be used because Serial uses pins 0 and 1,
// and we're using that for communication with the computer
SoftwareSerial serial(18,19);	
RoboClaw roboclaw(&serial,10000);

//This is a test. I don't belong here hehe

// Addresses (in hexadecimal) of each of the 4 motor controllers
#define left_front 0x81
#define right_front 0x82
#define left_rear 0x83
#define right_rear 0x84

// Max and min speed constraints
# define MAX_FORWARD 10.0 // units: m/s
# define MAX_TURN_RATE 6.0 // units: rad/s

void setup() {
  // Open roboclaw serial ports
  // This is the baud rate of the motor controllers, so it must be used
  roboclaw.begin(115200);

  // Begin serial communication with computer
  Serial.begin(9600);

  // Send message saying, "I'm ready!"
  Serial.write('R');

}

/******************************************************************************
NOIE: To run a motor forward at a certain speed, use the following command:

roboclaw.ForwardM1(address of motor, speed from 0 to 128);

To run it backwards, use this command:

roboclaw.BackwardM1(address of motor, speed from 0 to 128);

To run it forwards or backwards, use this command:

roboclaw.ForwardBackwardM1(address of motor, speed from 0 to 128);
 > 0 to 63 runs it backwards
 > 64 stops motor
 > 65 to 128 runs it forwards

Once the speed has been set, the motor will continue at that speed until it
receives another command.
******************************************************************************/

int speed;
String speed_forward_str;
float speed_forward;

int difference;
String speed_turn_str;
float speed_turn;

int final_speed_right;
int final_speed_left;

void loop() {

  // Get speed_forward and speed_turn from computer if there is stuff in the input buffer
  if (Serial.available() > 0)
  {

    // Read one line, expecting to receive speed_forward
    speed_forward_str = Serial.readStringUntil('\n');

    // Read another line, expecting to receive speed_turn
    speed_turn_str = Serial.readStringUntil('\n');

    // Convert both strings to floats
    speed_forward = speed_forward_str.toFloat();
    speed_turn = speed_turn_str.toFloat();

    // Constrain them to max and min velocities
    speed_forward = constrain(-MAX_FORWARD, MAX_FORWARD);
    speed_turn = constrain(-MAX_TURN_RATE, MAX_TURN_RATE);

    // Map the velocity input to values usable by the motor controllers
    speed = map_float(speed_forward, -MAX_FORWARD, MAX_FORWARD, 0, 128);
    difference = map_float(speed_turn, -MAX_TURN_RATE, MAX_TURN_RATE, -40, 40);

    /* Not sure if we'll need this bit:
    // Create dead zone in the middle of the stick so the robot will stop moving
    // when stick is in the middle
    if (speed >= 62 && speed <= 66)
    {
        speed = 64;
    }
    if (difference >= -1 && difference <= 1)
    {
        difference = 0;
    }
    */

    final_speed_left = speed - difference;
    final_speed_right = speed + difference;

    roboclaw.ForwardBackwardM1(left_front,  final_speed_left);
    roboclaw.ForwardBackwardM1(right_front, final_speed_right);
    roboclaw.ForwardBackwardM1(left_rear,   final_speed_left);
    roboclaw.ForwardBackwardM1(right_rear,  final_speed_right);
    
  }
  // If we aren't actively receiving input from the computer,
  else
  {

    // Turn all motors off
    roboclaw.ForwardM1(left_front,  0);
    roboclaw.ForwardM1(right_front, 0);
    roboclaw.ForwardM1(left_rear,   0);
    roboclaw.ForwardM1(right_rear,  0);

  }

}

// I copied Arduino's built-in map() function but made it use floats instead of ints
double map_float(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}