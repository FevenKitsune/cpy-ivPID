import time
import board
import math
from random import randint
from analogio import AnalogIn
from adafruit_motorkit import MotorKit

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
    # Motor throttle to use. Higher = faster, but oscillates more. Lower = slower, but oscillates less.
    motor_throttle = 0.6
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
        
        # While the elapsed time is less than convergence_time, loop.
        while time.monotonic() - start_time < convergance_time:
            # Retrieve sensor data and decimate 16-bit precision to 0-1023 (10-bit)
            servo_value = math.ceil(servo_feedback.value / 64)
            if servo_value < set_point:
                # If the servo is under the set_point then set throttle to negative.
                kit.motor3.throttle = -motor_throttle
            elif servo_value > set_point:
                # If the servo is above the set_point then set throttle to positive.
                kit.motor3.throttle = motor_throttle
            else:
                # If the servo is on the set_point, set throttle to zero.
                kit.motor3.throttle = 0.0

            # Print the measured value versus the set_point.
            print((servo_value, set_point))
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
