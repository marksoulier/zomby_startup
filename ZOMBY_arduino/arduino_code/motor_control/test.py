# This is a test to see if the arduino code for the zomby robot works.

# import serial library
# if you don't already have pySerial installed on your machine, go into the terminal
# and type: 
#   - Linux: python3 -m pip install pyserial
#   - Windows: python -m pip install pyserial
import serial
from time import sleep

# begin serial communication at baud of 9600
# on raspberry pi:
#   run ls dev/tty* and see if there's an "ACMX" or "USBX"
#   and change the serial address below accordingly.
ser = serial.Serial('COM6', 9600, timeout=1)
ser.reset_input_buffer()

arduino_ready_signal = 'R'

waiting = True

print("Waiting...")

# waits for the arduino to send a character indicating that it's ready to transmit data
while waiting:
    if ser.in_waiting > 0:
        char = ser.read(1).decode('utf-8')
        if char == arduino_ready_signal:
            print("Arduino is ready.")
            waiting = False
            
print("Done waiting.")

def send_speed_to_arduino(speed_forward, speed_turn):
    ser.write(bytes(str(speed_forward), "utf-8"))
    ser.write(b"\n")
    ser.write(bytes(str(speed_turn), "utf-8"))
    ser.write(b"\n")

delay = 0.1

max_speed_forward = 10.0  # units: m/s
max_speed_turn = 6.0      # units: rad/s

while True:
    # slowly increase speed to max
    x = 0
    while x <= max_speed_forward:
        send_speed_to_arduino(x, 0)
        sleep(delay)
        x = x + 0.1

    # slowly decrease speed to 0
    x = max_speed_forward
    while x >= 0.0:
        send_speed_to_arduino(x, 0)
        sleep(delay)
        x = x - 0.1

    # slowly increase spin to max
    x = 0
    while x <= max_speed_turn:
        send_speed_to_arduino(0, x)
        sleep(delay)
        x = x + 0.1

    # slowly decrease spin to 0
    x = max_speed_turn
    while x >= 0.0:
        send_speed_to_arduino(0, x)
        sleep(delay)
        x = x - 0.1
