from Motor import Motor
from Motor import wait_for_arduino
import serial
from time import sleep

if __name__ == "__main__":

    # begin serial communication with arduino
    mega_port = serial.Serial('COM9', 9600)

    # wait for arduino to be ready to receive data
    wait_for_arduino(mega_port)

    #motor_right = Motor(b"r", mega_port)
    motor_left = Motor(b"l", mega_port)

    #motor_right.setSlew(True)
    motor_left.setSlew(True)

    #motor_right.setSlewRate(10)
    motor_left.setSlewRate(10)

    #motor_right.setSpeed(96)
    motor_left.setSpeed(68)
    sleep(10)
    
    #motor_right.setSpeed(64)
    motor_left.setSpeed(64)
    sleep(10)

    