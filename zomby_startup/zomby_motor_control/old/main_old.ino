#include <RC_Receiver.h>

RC_Receiver receiver(13,11,9,7,5);
/* Ch1: 13
 * Ch2: 11
 * Ch3: 9
 * Ch4: 7
 * Ch5: 5
 */

//Channel min and max value
//Leave the default value for the un used channels
//Invert the min and max val to reverse
int minMax[6][2] = { 
//   Min   Max
	{1088,1880},  // Ch1: Throttle
	{1088,1873},  // Ch2: Elevator
	{1085,1871},  // Ch3: Aileron (Max to the left and min to the right)
	{1090,1875},  // Ch4: Rudder (Max to the left and min to the right)
    {1880,1080},  // Ch5: Gear Channel (kill switch) (0 = 1880) (1 = 1080)
    {1880,1080}   // Ch6: Aux channel (0 = 1880) (1/2 = 1480) (1 = 1080)
};

#define ELEV_CH 2
#define AIL_CH 3
#define GEAR_CH 5
#define AUX_CH 6

// Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

// See limitations of Arduino SoftwareSerial
// Rx pin: 18
// TX pin: 19
SoftwareSerial serial(18,19);	
RoboClaw roboclaw(&serial,10000);

// Addresses (in hexadecimal) of each of the 4 motor controllers
#define left_front 0x81
#define right_front 0x82
#define left_rear 0x83
#define right_rear 0x84

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

// Motor slew rate delay in ms
int motor_slew_delay = 20;

void setup() {
    
    // Begin serial communication with motor controllers
    roboclaw.begin(115200);

    // Set min and max values for receiver channels
    receiver.setMinMax(minMax);

    // Begin serial communication with computer
    Serial.begin(9600);

    // Send message to computer saying, "I'm ready!"
    Serial.write('R');

}



void loop() {

    // Get RC switch value
    int kill_switch = receiver.getMap(GEAR_CH);

    // If aux switch is on
    if (kill_switch > 70) {

        // Let the RC Receiver control the robot
        RC_control();

    }

    // Else if the aux switch is off
    else if (kill_switch < 30) {

        // Let the computer control the robot
        computer_control();

    }

    // Else if the aux switch is in the middle,
    else {

        // Turn on red LED or something

    }
    
}

void RC_control() {

    int speed = map(receiver.getMap(ELEV_CH), 0, 100, 0, 128);
    int difference = map(receiver.getMap(AIL_CH), 0, 100, -40, 40);

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

    int final_speed_left = speed - difference;
    int final_speed_right = speed + difference;

    roboclaw.ForwardBackwardM1(left_front,  final_speed_left);
    roboclaw.ForwardBackwardM1(right_front, final_speed_right);
    roboclaw.ForwardBackwardM1(left_rear,   final_speed_left);
    roboclaw.ForwardBackwardM1(right_rear,  final_speed_right);

}

void computer_control() {

    byte id, desired_speed;
    byte desired_speed_left, desired_speed_right;
    static byte actual_speed_left = 64;
    static byte actual_speed_right = 64;

    // If there are 4 bytes in the input buffer 
    if (Serial.available() >= 4) {

        // Loop two times
        for (int i = 0; i < 2; i++) {

            // Read two bytes. First is motor ID and second is speed
            id = Serial.read();
            desired_speed = Serial.read();

            // If received speed is for the right motor
            if (id == 'r') {

                desired_speed_right = desired_speed;

            }

            // If received speed is for the left motor
            else if (id == 'l') {

                desired_speed_left = desired_speed;

            }

        }

    }

    // If we don't get anything from the computer
    else {

        // Set desired speed to stopped
        desired_speed_left = 64;
        desired_speed_right = 64;

    }

    // Increment actual speed towards desired speed
    // 1 tick every x amount of miliseconds
    if (actual_speed_left < desired_speed_left) {

        actual_speed_left++;

    }
    else if (actual_speed_left > desired_speed_left) {

        actual_speed_left--;

    }
    if (actual_speed_right < desired_speed_right) {

        actual_speed_right++;

    }
    else if (actual_speed_right > desired_speed_right) {

        actual_speed_right--;

    }

    // Set left motor speeds
    roboclaw.ForwardBackwardM1(left_front,  actual_speed_left);
    roboclaw.ForwardBackwardM1(left_rear,   actual_speed_left);

    // Set right motor speeds
    roboclaw.ForwardBackwardM1(right_front, actual_speed_right);
    roboclaw.ForwardBackwardM1(right_rear,  actual_speed_right);
    
    // Motor slew delay
    delay(motor_slew_delay);

}