import time
import board
import math
from analogio import AnalogIn
from adafruit_motorkit import MotorKit
from adafruit_simplemath import map_range
import PID

# Main PID controller
PID_loop = PID.PID(21, 1.6, 0.195)
"""
Test rig settings:
5v:
P = 7.3
6v:
just P = 4.65
PID = 21, 1.6, 0.04/0.195
"""

# Clamp the maximum and minimum values reported by the PID loop to control range translation to the throttle.
PID_RANGE = 1536
# Motor driver object
kit = MotorKit()
# Servo data
servo_feedback = AnalogIn(board.A3)


"""
Main function
"""


def main():
    # main() setup code
    kit.motor3.throttle = 0.0   # Stop the motor on startup.
    PID_loop.setSetPoint(200)   # 10-bit 0 - 1023

    """Timers:
    These timers run sections of code at an interval without interrupting the rest of the code.
    """
    timer1_interval = 0.01                      # Number of seconds between each execution for timer1.
    # Set the last execution to the current time.
    timer1_last_execution = time.monotonic()
    # Number of seconds between each execution for timer2.
    timer2_interval = 0.005
    # Set the last execution to the current time.
    timer2_last_execution = time.monotonic()

    loop_count = 0  # Tracks the number of task loop iterations.
    tick_tock = 0   # Execution tracker that steps between positions.

    """Primary task loop!
    This will run indefinitely.
    """
    while True:
        # Decimate 16-bit precision to 0-1023 (10-bit)
        servo_value = math.ceil(servo_feedback.value / 64)
        # Update the PID loop with the decimated value.
        PID_loop.update(servo_value)
        # Map the PID output value to a throttle value from -1 to 1. Clamped with the PID_RANGE property.
        new_value = -map_range(PID_loop.control_variable, -
                               PID_RANGE, PID_RANGE, -1, 1)
        # Update the motor with the new throttle value.
        kit.motor3.throttle = new_value

        """Timer1 Block
        This block of code runs at the interval determined by timer1_interval
        """
        timer1_delta = time.monotonic() - \
            timer1_last_execution  # Update delta time since last timer1 function execution.
        # If the time since last execution is greater than the interval, run the following functions.
        if timer1_delta > timer1_interval:
            # Print data values for graphing.
            print((servo_value, PID_loop.set_point))
            # Since timer1 has been executed, update the last_execution.
            timer1_last_execution = time.monotonic()

        """Timer2 Block
        This block of code runs at the interval determined by timer2_interval
        """
        timer2_delta = time.monotonic() - \
            timer2_last_execution  # Update delta time since last timer2 function execution.
        # If the time since last execution is greater than the interval, run the following functions.
        if timer2_delta > timer2_interval:
            # Run routine that generates the next desired position.
            generateNewTarget()
            timer2_last_execution = (
                time.monotonic()
            )                           # Since timer2 has been executed, update the last_execution.

        loop_count += 1  # Update loop_count to track iterations over time.


VALUE_MAX = 923
VALUE_MIN = 100
CYCLES = 1


def generateNewTarget():
    sample = time.monotonic()
    PID_loop.setSetPoint(
        (1 / 2)
        * (VALUE_MAX - VALUE_MIN)
        * math.sin((2 * math.pi * CYCLES * sample) / 2)
        + (VALUE_MAX / 2)
        + (VALUE_MIN / 2)
    )


"""
Putting all your main code within their own function, then using this block of code
ensures all functions are declared prior to inital execution.
"""
if __name__ == "__main__":
    main()
