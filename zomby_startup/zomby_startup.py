# Here, I import the pySerial library so we can communicate with Arduino
import serial

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from zomby_motor_control.Zomby import Zomby

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('robot controller')

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        

        self.cmd_vel_sub  # prevent unused variable warning

        self.Zomby = Zomby.Zomby("COM9", 9600)
        self.Zomby.forward(50)
        

    def listener_callback(self, msg: Twist) -> None:
        self.get_logger().info(f'I heard velocity: '+ str(msg.linear.x))
        self.get_logger().info(f'I heard angular_velocity: '+ str(msg.angular.z))


        velocity = msg.linear.x
        omega = msg.angular.z
        self.send_speed_to_arduino(velocity, omega)





    def send_speed_to_arduino(self, velocity, omega):

        ########################################ENTER YOUR CODE HERE#############################################
        #variables speed_forward and speed_turn are floats.
        #We will need a way to send those floats to the arduino over usb cable
        #speed_forward is also linear velocity
        #speed_turn is also angular velocity
        #list out any packages you need to do this in the comments because I will need to add them elsewhere in the package
        
        # Radius of the wheel
        r =  0.2

        # Distance between wheels
        L =  0.6

        # Control vector [velocity, omega]
        control = np.array([velocity, omega])

        # Matrix M
        M = np.array([[r/2, r/2], [r/L, -r/L]])

        # Calculate the inverse of M
        M_inv = np.linalg.inv(M)

        # Calculate the wheel speeds
        wheel_speeds = np.dot(M_inv, control)

        # Extract the right and left wheel speeds
        speed_right = wheel_speeds[0]
        speed_left = wheel_speeds[1]

        # Now, you can set the calculated speeds to your robot
        self.Zomby.setSpeed(speed_right, speed_left)
    
	

def main(args=None):
    rclpy.init(args=args)

    robot_control = MinimalSubscriber()

    print("startup node running")
    rclpy.spin(robot_control)

    robot_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':    
    main()
