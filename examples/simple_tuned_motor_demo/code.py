import time
import board
import math
from analogio import AnalogIn
from adafruit_motorkit import MotorKit
from adafruit_simplemath import map_range
import PID

# TODO: Comment this code. Make it clear where the sensor is reading data, and where the output is set.

PID_loop = PID.PID(21, 1.6, 0.195)  # Main PID controller
"""
Test rig settings:
5v:
P = 7.3
6v:
just P = 4.65
PID = 21, 1.6, 0.04 for small movements, ~0.5 for large movements 0.195?
"""

PID_RANGE = 1536                    # Clamp the maximum and minimum values reported by the PID loop to control range translation to the throttle.
kit = MotorKit()                    # Motor driver object
servo_feedback = AnalogIn(board.A3) # Servo data


"""
Main function
"""


def main():
    # main() setup code
    kit.motor3.throttle = 0.0  # Stop the motor on startup.
    PID_loop.setSetPoint(200)  # 10-bit 0 - 1023

    """Primary task loop!
    This will run indefinitely.
    """
    while True:
        servo_value = math.ceil(servo_feedback.value / 64)  # Decimate 16-bit precision to 0-1023 (10-bit)
        PID_loop.update(servo_value)                        # Update the PID loop with the decimated value.
        new_value = -map_range(                             # Map the PID output value to a throttle value from -1 to 1. Clamped with the PID_RANGE property.
            PID_loop.control_variable,
            -PID_RANGE, PID_RANGE,
            -1, 1
        )
        kit.motor3.throttle = new_value                     # Update the motor with the new throttle value.
        print((servo_value, PID_loop.set_point))            # Print the measured value versus the set_point.
        time.sleep(0.01)                                    # Wait 0.01 seconds.


"""
Putting all your main code within their own function, then using this block of code
ensures all functions are declared prior to inital execution.
"""
if __name__ == "__main__":
    main()
