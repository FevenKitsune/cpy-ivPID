import time
import board
import math
from random import randint
from analogio import AnalogIn
from adafruit_motorkit import MotorKit
from adafruit_simplemath import map_range
import PID

# Main PID controller
PID_loop = PID.PID(21, 0.4, 0.195)
# Test rig settings:
# 5v:
# P = 7.3
# 6v:
# just P = 4.65
# PID = 21, 1.6/0.4, 0.04 for small movements, ~0.5 for large movements 0.195?

# Clamp the maximum and minimum values reported by the PID loop to control range translation to the throttle.
PID_RANGE = 1536
# Motor driver object
kit = MotorKit()
# Servo data
servo_feedback = AnalogIn(board.A3)


def main():
    """The main function. Good practice to have a defined starting point.
    Called at the end of the code.

    Keyword arguments:
    None
    """
    # Stop the motor on startup.
    kit.motor3.throttle = 0.0
    # Minimum random set_point.
    random_min = 200
    # Maximum random set_point.
    random_max = 823
    # 10-bit 0 - 1023
    PID_loop.setSetPoint(200)
    # Initalizer for start_time variable.
    start_time = 0.0
    # How long the controller should attempt to converge on the set_point.
    convergance_time = 5.0

    # Primary task loop, this runs indefinitely.
    while True:
        print("Finding new target point...")
        # Find a new random set_point.
        set_point = randint(random_min, random_max)
        # Announce new set_point.
        print(
            f"Target is {set_point}\n"
            f"Starting convergence in 5 seconds..."
        )
        # Wait 5 seconds before beginning convergence.
        time.sleep(5.0)
        # Start convergence timer.
        start_time = time.monotonic()
        # Clear terms from previous cycle. Set after delay to prevent windup.
        PID_loop.clear()
        # Set point to calculated value.
        PID_loop.setSetPoint(set_point)
        
        # While the elapsed time is less than convergence_time, loop.
        while time.monotonic() - start_time < convergance_time:
            # Retrieve sensor data and decimate 16-bit precision to 0-1023 (10-bit)
            servo_value = math.ceil(servo_feedback.value / 64)
            # Update the PID loop with the decimated value.
            PID_loop.update(servo_value)
            # Map the PID output value to a throttle value from -1 to 1. Clamped with the PID_RANGE property.
            new_value = -map_range(
                PID_loop.control_variable,  # The output from the PID loop.
                -PID_RANGE, PID_RANGE,      # The min/max expected from the PID loop.
                -1, 1                       # The min/max of the device controlled by the PID loop.
            )
            # Update the motor with the new throttle value.
            kit.motor3.throttle = new_value
            # Print the measured value versus the set_point.
            print((servo_value, PID_loop.set_point))
            # Wait 0.01 seconds before trying again.
            time.sleep(0.01)

        # Once convergence timer has expired, stop motor.
        kit.motor3.throttle = 0.0
        # Announce completion and loop back to beginning.
        print("Convergence complete.")


if __name__ == "__main__":
    # Putting all your main code within their own function, then using this block of code
    # ensures all functions are declared prior to inital execution.
    main()
