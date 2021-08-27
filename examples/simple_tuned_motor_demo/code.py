import time
import board
import math
from random import randint
from analogio import AnalogIn
from adafruit_motorkit import MotorKit
from adafruit_simplemath import map_range
import PID


PID_loop = PID.PID(21, 1.6, 0.04)  # Main PID controller
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
    kit.motor3.throttle = 0.0   # Stop the motor on startup.
    random_min = 200            # Minimum random set_point.
    random_max = 823            # Maximum random set_point.
    PID_loop.setSetPoint(200)   # 10-bit 0 - 1023
    start_time = 0.0            # Initalizer for start_time variable.
    convergance_time = 5.0      # How long the controller should attempt to converge on the set_point.

    """Primary task loop!
    This will run indefinitely.
    """
    while True:
        print("Finding new target point...")
        set_point = randint(random_min, random_max) # Find a new random set_point.
        PID_loop.setSetPoint(set_point)
        print(                                      # Announce new set_point.
            f"Target is {set_point}\n"
            f"Starting convergence in 5 seconds..."
        )
        time.sleep(5.0)                             # Wait 5 seconds before beginning convergence.
        start_time = time.monotonic()               # Start convergence timer.
        
        while time.monotonic() - start_time < convergance_time: # While the elapsed time is less than convergence_time, loop.
            servo_value = math.ceil(servo_feedback.value / 64)  # Retrieve sensor data and decimate 16-bit precision to 0-1023 (10-bit)
            PID_loop.update(servo_value)                        # Update the PID loop with the decimated value.
            new_value = -map_range(                             # Map the PID output value to a throttle value from -1 to 1. Clamped with the PID_RANGE property.
                PID_loop.control_variable,  # The output from the PID loop.
                -PID_RANGE, PID_RANGE,      # The min/max expected from the PID loop.
                -1, 1                       # The min/max of the device controlled by the PID loop.
            )
            kit.motor3.throttle = new_value                     # Update the motor with the new throttle value.
            print((servo_value, PID_loop.set_point))            # Print the measured value versus the set_point.
            time.sleep(0.01)                                    # Wait 0.01 seconds before trying again.

        kit.motor3.throttle = 0.0       # Once convergence timer has expired, stop motor.
        print("Convergence complete.")  # Announce completion and loop back to beginning.


"""
Putting all your main code within their own function, then using this block of code
ensures all functions are declared prior to inital execution.
"""
if __name__ == "__main__":
    main()
