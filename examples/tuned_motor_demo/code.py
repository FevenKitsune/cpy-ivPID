import time
import board
import pwmio
import math
import microcontroller
from analogio import AnalogIn
from adafruit_motorkit import MotorKit
from adafruit_simplemath import map_range
import PID

# Main PID controller
PID_loop = PID.PID(21, 1.6, 0.17)
"""
Test rig settings:
5v:
P = 7.3
6v:
just P = 4.65
PID = 21, 1.6, 0.04/0.17
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
    timer1_last_execution = time.monotonic()    # Set the last execution to the current time.
    timer2_interval = 1                         # Number of seconds between each execution for timer2.
    timer2_last_execution = time.monotonic()    # Set the last execution to the current time.

    loop_count = 0  # Tracks the number of task loop iterations.
    tick_tock = 0   # Execution tracker that steps between positions.
    
    """Primary task loop!
    This will run indefinitely.
    """
    while True:
        servo_value = math.ceil(servo_feedback.value / 64)                      # Decimate 16-bit precision to 0-1023 (10-bit)
        PID_loop.update(servo_value)                                            # Update the PID loop with the decimated value.
        new_value = -map_range(PID_loop.control_variable, -PID_RANGE, PID_RANGE, -1, 1)   # Map the PID output value to a throttle value from -1 to 1. Clamped with the PID_RANGE property.
        kit.motor3.throttle = new_value                                         # Update the motor with the new throttle value.

        """Timer1 Block
        This block of code runs at the interval determined by timer1_interval
        """
        timer1_delta = time.monotonic() - timer1_last_execution # Update delta time since last timer1 function execution.
        if timer1_delta > timer1_interval:                      # If the time since last execution is greater than the interval, run the following functions.
            print((servo_value, PID_loop.set_point))            # Print data values for graphing.
            timer1_last_execution = time.monotonic()            # Since timer1 has been executed, update the last_execution.

        """Timer2 Block
        This block of code runs at the interval determined by timer2_interval
        """
        timer2_delta = time.monotonic() - timer2_last_execution # Update delta time since last timer2 function execution.
        if timer2_delta > timer2_interval:                      # If the time since last execution is greater than the interval, run the following functions.
            """Not the most elegant solution but it works."""
            if tick_tock == 0:              # Step 0
                PID_loop.setSetPoint(100)   
                tick_tock = 1               # Set next step to Step 1
            elif tick_tock == 1:            # Step 1
                PID_loop.setSetPoint(500)
                tick_tock = 2               # Set next step to Step 2
            elif tick_tock == 2:            # Step 2
                PID_loop.setSetPoint(900)
                tick_tock = 3               # Set next step to Step 3
            elif tick_tock == 3:            # Step 3
                PID_loop.setSetPoint(500)
                tick_tock = 0               # Set next step to Step 1.
            timer2_last_execution = time.monotonic()            # Since timer2 has been executed, update the last_execution.
        
        loop_count += 1 # Update loop_count to track iterations over time.


"""
Putting all your main code within their own function, then using this block of code
ensures all functions are declared prior to inital execution.
"""
if __name__ == "__main__":
    main()
