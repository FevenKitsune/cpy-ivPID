import time
import board
import math
from random import randint
from analogio import AnalogIn
from adafruit_motorkit import MotorKit

# TODO: Comment this code. Make it clear where the sensor is reading data, and where the output is set.

# Motor driver object
kit = MotorKit()
# Servo data
servo_feedback = AnalogIn(board.A3)


"""
Main function
"""
def main():
    # main() setup code
    kit.motor3.throttle = 0.0  # Stop the motor on startup.
    set_point = 200
    random_min = 200
    random_max = 823
    motor_throttle = 1.0

    start_time = 0.0
    convergance_time = 5.0

    """Primary task loop!
    This will run indefinitely.
    """
    while True:
        print("Finding new target point...")
        set_point = randint(random_min, random_max)
        print(f"Target is {set_point}")
        print("Starting convergence in 5 seconds...")
        time.sleep(5.0)
        start_time = time.monotonic()
        while time.monotonic() - start_time < convergance_time:
            servo_value = math.ceil(
                servo_feedback.value / 64
            )  # Decimate 16-bit precision to 0-1023 (10-bit)
        
            if servo_value < set_point:
                kit.motor3.throttle = -motor_throttle
            elif servo_value > set_point:
                kit.motor3.throttle = motor_throttle
            else:
                kit.motor3.throttle = 0.0

            print((servo_value, set_point))
            time.sleep(0.01)
        
        kit.motor3.throttle = 0.0
        print("Convergence complete.")

        
        


"""
Putting all your main code within their own function, then using this block of code
ensures all functions are declared prior to inital execution.
"""
if __name__ == "__main__":
    main()
