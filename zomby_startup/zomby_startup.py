import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String


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
        hello = "hello" #delete this line








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
    main()