"""
A direct CircuitPython reimplementation of Ivmech Mechatronics ivPID Python Library
Original source: https://github.com/ivmech/ivPID/blob/master/PID.py

Reimplementation and documentation by: Will S.
"""

import time

"""Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller in the Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller
"""


class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        """Constructor
        A constructor is the first function called when an object is created, and sets up the
        instance of the object.

        Keyword arguments:
        P -- the proportional weight assigned on object creation. (default 0.2)
        I -- the integral weight assigned on object creation. (default 0.0)
        D -- the derivative weight assigned on object creation. (default 0.0)
        """

        """self.* variables are instance variables and are unique to the instance of the object."""
        self.Kp = P  # The instance variable for P weight
        self.Ki = I  # The instance variable for I weight
        self.Kd = D  # The instance variable for D weight

        """Variables assigned to a null value are typically assigned later in the program."""
        self.Pterm = 0.0  # The computed P term.
        self.Iterm = 0.0  # The computed I term.
        self.Dterm = 0.0  # The computed D term.

        # set_point tracks the desired target output.
        self.set_point = 0.0
        # Initialize the current_time with current time.
        self.current_time = time.monotonic()
        # last_time caches the previous time update() was called to compute time delta.
        self.last_time = self.current_time
        # last_error caches the error from the prevous time update() was called to compute error delta.
        self.last_error = 0.0

        # control_variable is the output determined by the PID controller.
        self.control_variable = 0.0

    def clear(self):
        """Clear all PID computations and coefficients.
        
        """
        self.set_point = 0.0    # Clear all terms!
        self.Pterm = 0.0
        self.Iterm = 0.0
        self.Dterm = 0.0
        self.last_error = 0.0
        self.control_variable = 0.0

    def update(self, process_variable):
        """Update the PID controller with the measured process variable and compute the control variable.

        Keyword arguments:
        process_variable -- number input representing the measured process variable.
        """

        # Compute the error from the process_variable to the set_point.
        error = self.set_point - process_variable
        # Update current_time.
        self.current_time = time.monotonic()
        # Calculate the time delta since the previous update.
        delta_time = self.current_time - self.last_time
        # Calculate the error delta since the previous update.
        delta_error = error - self.last_error

        # Multiply the proportional constant with the error to compute the P-term.
        self.Pterm = self.Kp * error
        # Multiply the error by the time delta and add that value to the I-term.
        self.Iterm += error * delta_time
        # D-term is defaulted to 0.0 unless time has passed since the previous update.
        self.Dterm = 0.0
        if delta_time > 0:
            # If time has passed, update the D-term with the change of error (delta_error) over the change of time (delta_time)
            self.Dterm = delta_error / delta_time

        # Now that computation has finished, set last_time to the time used for current_time.
        self.last_time = self.current_time
        # Set last_error to the error computed.
        self.last_error = error

        # Compute all 3 terms of the PID formula.
        self.control_variable = self.Pterm + \
            (self.Ki * self.Iterm) + (self.Kd * self.Dterm)

    def setSetPoint(self, desired_set_point):
        """Change the PID controller set point.

        Keyword arguments:
        desired_set_point -- the value set_point will be updated to.
        """
        self.set_point = desired_set_point
