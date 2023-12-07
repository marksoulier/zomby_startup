// Updated as of 11/4/23

#include <RC_Receiver.h>

RC_Receiver receiver(13,12,11);
/* Ch1: 13
 * Ch2: 12
 * Ch3: 11
 * Ch4: 
 * Ch5: 
 */

//Channel min and max value
//Leave the default value for the un used channels
//Invert the min and max val to reverse
int minMax[3][2] = { 
//   Min   Max
	{1064,1907},  // Ch1: Elevator 
	{1065,1899},  // Ch2: Aileron
	{1064,1906}//,  // Ch3: Gear 
	//{1090,1875},  // Ch4:  
  //  {1880,1080},  // Ch5: 
  //  {1880,1080}   // Ch6: 
};

#define ELEV_CH 1
#define AIL_CH 2
#define GEAR_CH 3
// #define AUX_CH 6

#define MOTOR_ID_R 'r'
#define MOTOR_ID_L 'l'

// Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

// See limitations of Arduino SoftwareSerial
// Tx pin: 18
// RX pin: 19
SoftwareSerial serial(19,18);	
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

    // Serial.print("kill_switch value: ");
    // Serial.println(kill_switch);

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
    // Serial.print("Speed value: ");
    // Serial.println(speed);
    int difference = map(receiver.getMap(AIL_CH), 0, 100, 40, -40);
    // Serial.print("Difference value: ");
    // Serial.println(difference);

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

    byte id;
    byte speed_received;
    static byte speed_left = 64;
    static byte speed_right = 64;

    // If there are 4 bytes in the input buffer 
    if (Serial.available() >= 4) {

        // Loop two times
        for (int i = 0; i < 2; i++) {

            // Read two bytes. First is motor ID and second is speed
            id = Serial.read();
            speed_received = Serial.read();

            // If received speed is for the right motor
            if (id == MOTOR_ID_R) {

                speed_right = speed_received;
                
            }

            // If received speed is for the left motor
            else if (id == MOTOR_ID_L) {

                speed_left = speed_received;

            }

        }

        // Set right motor speeds
        roboclaw.ForwardBackwardM1(right_front, speed_right);
        roboclaw.ForwardBackwardM1(right_rear,  speed_right);

        // Set left motor speeds
        roboclaw.ForwardBackwardM1(left_front,  speed_left);
        roboclaw.ForwardBackwardM1(left_rear,   speed_left);

    }

}