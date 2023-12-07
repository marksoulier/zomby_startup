import serial
import threading
from time import sleep

DEBUG = 1

class Zomby:
    def __wait_for_arduino(self):

        # waits to receive this character
        arduino_ready_signal = 'R'
        
        waiting = True
        
        print("Waiting for Arduino...")

        while waiting:
            if self.__serial_port.in_waiting > 0:
                char = self.__serial_port.read(1).decode('utf-8')
                if char == arduino_ready_signal:
                    print("Arduino is ready.")
                    waiting = False


    def __init__(self, port, baud_rate):
        # Initialize serial port
        self.__serial_port = serial.Serial(port, baud_rate)

        # For use in self.__motor_control_function()
        self.__stop_motors = threading.Event()

        self.__desired_speed_right = 64
        self.__speed_right = 64

        self.__desired_speed_left = 64
        self.__speed_left = 64

        self.__motor_slew_delay = 4 / 64

        # For use in self.__sendSpeed()
        self.__ser_ID_right = b"r"
        self.__ser_ID_left = b"l"


        # Wait for arduino
        self.__wait_for_arduino()

        # Spin up motor control thread
        self.__motor_control = threading.Thread(target=self.__motor_control_function, daemon=True)
        self.__motor_control.start()

    
    def __sendSpeed(self): # sends motor speed attribute to arduino
        # send right motor ID to arduino
        self.__serial_port.write(self.__ser_ID_right)

        # DEBUG
        if DEBUG:
            print(self.__ser_ID_right)

        # send right motor speed to arduino
        self.__serial_port.write(self.__speed_right.to_bytes(length=1, byteorder='big'))

        # DEBUG
        if DEBUG:
            print(self.__speed_right)

        # send left motor ID to arduino
        self.__serial_port.write(self.__ser_ID_left)

        # DEBUG
        if DEBUG:
            print(self.__ser_ID_left)

        # send left motor speed to arduino
        self.__serial_port.write(self.__speed_left.to_bytes(length=1, byteorder='big'))

        # DEBUG
        if DEBUG:
            print(self.__speed_left)


    def __motor_control_function(self):
        while self.__stop_motors.is_set() == False:

            if self.__speed_right < self.__desired_speed_right:
                self.__speed_right += 1

            elif self.__speed_right > self.__desired_speed_right:
                self.__speed_right -= 1
                
            if self.__speed_left < self.__desired_speed_left:
                self.__speed_left += 1

            elif self.__speed_left > self.__desired_speed_left:
                self.__speed_left -= 1

            self.__sendSpeed()

            sleep(self.__motor_slew_delay)


    def setSlewRate(self, slew_time):
        self.__motor_slew_delay = slew_time / 64


    def turnLeft(self, percent_speed):
        if percent_speed > 100:
            percent_speed = 100
        elif percent_speed < 0:
            percent_speed = 0

        self.__desired_speed_left = 64 - int(64 * (percent_speed/100))
        self.__desired_speed_right = 64 + int(64 * (percent_speed/100))


    def turnRight(self, percent_speed):
        if percent_speed > 100:
            percent_speed = 100
        elif percent_speed < 0:
            percent_speed = 0
            
        self.__desired_speed_left = 64 + int(64 * (percent_speed/100))
        self.__desired_speed_right = 64 - int(64 * (percent_speed/100))

    
    def forward(self, percent_speed):
        if percent_speed > 100:
            percent_speed = 100
        elif percent_speed < 0:
            percent_speed = 0
            
        self.__desired_speed_left = 64 + int(64 * (percent_speed/100))
        self.__desired_speed_right = 64 + int(64 * (percent_speed/100))

    
    def backward(self, percent_speed):
        if percent_speed > 100:
            percent_speed = 100
        elif percent_speed < 0:
            percent_speed = 0
            
        self.__desired_speed_left = 64 - int(64 * (percent_speed/100))
        self.__desired_speed_right = 64 - int(64 * (percent_speed/100))
    

    def stop(self):
        self.__desired_speed_left = 64
        self.__desired_speed_right = 64

    
    # currently can't be undone when called
    def stopMotorControl(self):
        self.__stop_motors.set()