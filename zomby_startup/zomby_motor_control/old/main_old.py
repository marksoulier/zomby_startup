import serial
import threading
from time import sleep

# begin serial communication with arduino
ser = serial.Serial('COM6', 9600)

# THIS MUST MATCH THE MOTOR SLEW DELAY OF THE ARDUINO
# units are in seconds
motor_slew_delay = 0.02

class Motor:
    # default speed is stopped (64)
    speed = 64

    # sets motor speed attribute
    def setSpeed(self, speed_to_set):
        self.speed = speed_to_set

        # constrain motor speed to value from 0 to 128
        if self.speed < 0:
            self.speed = 0
        if self.speed > 128:
            self.speed = 128

    # constructor
    def __init__(self, ID, ser):
        # initialize motor ID
        self.ID = ID
        # pass reference to serial object
        self.ser = ser

    # sends motor speed attribute to arduino
    def sendSpeed(self):
        # send motor ID to arduino
        self.ser.write(self.ID)

        # send motor speed to arduino
        self.ser.write(self.speed.to_bytes(length=1, byteorder='big'))

# waits for arduino to indicate that it's ready
def wait_for_arduino():

    # waits to receive this character
    arduino_ready_signal = 'R'
    
    waiting = True
    
    print("Waiting...")

    while waiting:
        if ser.in_waiting > 0:
            char = ser.read(1).decode('utf-8')
            if char == arduino_ready_signal:
                print("Arduino is ready.")
                waiting = False
                
    print("Done waiting.")

motor_right = Motor(b"r", ser)
motor_left = Motor(b"l", ser)

def send_motor_stream():
    while True:
        # send motor speeds to arduino
        motor_right.sendSpeed()
        motor_left.sendSpeed()
        # wait
        sleep(motor_slew_delay)

# -------------------------------------
# Here's the main part of the program:
# -------------------------------------

if __name__ == "__main__":

    # wait for arduino to be ready to receive data
    wait_for_arduino()

    # start parallel process that sends a constant stream of motor speed data
    # it will shutdown when the main part of the program stops because of "daemon" option
    x = threading.Thread(target=send_motor_stream, args=(1,), daemon=True)
    x.start()

    # spin motors as fast as they will from stopped to full blast
    motor_right.setSpeed(128)
    motor_left.setSpeed(128)
    sleep(4)

    # spin motors as fast as they will from full blast to stopped
    motor_right.setSpeed(64)
    motor_left.setSpeed(64)
    sleep(4)

    # spin motors smoothly from stopped to full blast
    for x in range(64, 128):
        motor_right.setSpeed(x)
        motor_left.setSpeed(x)
        # wait in seconds
        sleep(0.04)

    # wait for a few seconds
    sleep(4)

    # spin motors smoothly from full blast to stopped
    for x in range(128, 64):
        motor_right.setSpeed(x)
        motor_left.setSpeed(x)
        # wait 20 ms
        sleep(0.04)

    # wait for a few seconds
    sleep(4)