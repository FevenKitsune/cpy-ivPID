import time
import board
import math
from random import randint
from analogio import AnalogIn
from adafruit_motorkit import MotorKit


kit = MotorKit()                    # Motor driver object
servo_feedback = AnalogIn(board.A3) # Servo data


"""
Main function
"""
def main():
    """The main function. Good practice to have a defined starting point.
    Called at the end of the code.

    Keyword arguments:
    None
    """
    kit.motor3.throttle = 0.0   # Stop the motor on startup.
    random_min = 200            # Minimum random set_point.
    random_max = 823            # Maximum random set_point.
    motor_throttle = 0.6        # Motor throttle to use. Higher = faster, but oscillates more. Lower = slower, but oscillates less.
    start_time = 0.0            # Initalizer for start_time variable.
    convergance_time = 5.0      # How long the controller should attempt to converge on the set_point.

    """Primary task loop!
    This will run indefinitely.
    """
    while True:
        print("Finding new target point...")
        set_point = randint(random_min, random_max) # Find a new random set_point.
        print(                                      # Announce new set_point.
            f"Target is {set_point}\n"
            f"Starting convergence in 5 seconds..."
        )
        time.sleep(5.0)                             # Wait 5 seconds before beginning convergence.
        start_time = time.monotonic()               # Start convergence timer.
        
        while time.monotonic() - start_time < convergance_time: # While the elapsed time is less than convergence_time, loop.
            servo_value = math.ceil(servo_feedback.value / 64)  # Retrieve sensor data and decimate 16-bit precision to 0-1023 (10-bit)
            if servo_value < set_point:
                kit.motor3.throttle = -motor_throttle           # If the servo is under the set_point then set throttle to negative.
            elif servo_value > set_point:
                kit.motor3.throttle = motor_throttle            # If the servo is above the set_point then set throttle to positive.
            else:
                kit.motor3.throttle = 0.0                       # If the servo is on the set_point, set throttle to zero.
            print((servo_value, set_point))                     # Print the measured value versus the set_point.
            time.sleep(0.01)                                    # Wait 0.01 seconds before trying again.

        kit.motor3.throttle = 0.0       # Once convergence timer has expired, stop motor.
        print("Convergence complete.")  # Announce completion and loop back to beginning.


"""
Putting all your main code within their own function, then using this block of code
ensures all functions are declared prior to inital execution.
"""
if __name__ == "__main__":
    main()
