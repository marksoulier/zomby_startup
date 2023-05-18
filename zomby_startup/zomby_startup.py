# Here, I import the pySerial library so we can communicate with Arduino
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

# Here, I open the serial port and reset the input buffer
# Run "ls /dev/tty*" and see if "/dev/ttyACMX" or "/dev/ttyUSBX" is listed and enter that below
ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.cmd_vel_sub  # prevent unused variable warning

    def listener_callback(self, msg: Twist) -> None:
        self.get_logger().info(f'I heard velocity: '+ str(msg.linear.x))
        self.get_logger().info(f'I heard angular_velocity: '+ str(msg.angular.z))


        speed_forward = msg.linear.x
        speed_turn = msg.angular.z
        self.send_speed_to_arduino(speed_forward, speed_turn)



    def send_speed_to_arduino(self, speed_forward, speed_turn):

        ########################################ENTER YOUR CODE HERE#############################################
        #variables speed_forward and speed_turn are floats.
        #We will need a way to send those floats to the arduino over usb cable
        #speed_forward is also linear velocity
        #speed_turn is also angular velocity
        #list out any packages you need to do this in the comments because I will need to add them elsewhere in the package
	ser.write(bytes(str(speed_forward), "utf-8"))
	ser.write(b"\n")
	ser.write(bytes(str(speed_turn), "utf-8"))
	ser.write(b"\n")

# Wait for arduino to be ready to receive serial messages
def wait_for_arduino():
	arduino_ready_signal = 'R'

	waiting = True

	#print("Waiting...")

	# waits for the arduino to send a character indicating that it's ready to transmit data
	while waiting:
	    if ser.in_waiting > 0:
		char = ser.read(1).decode('utf-8')
		if char == arduino_ready_signal:
		    #print("Arduino is ready.")
		    waiting = False
		    
	#print("Done waiting.")
	

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    ser.reset_input_buffer()
    wait_for_arduino()
    
    main()
