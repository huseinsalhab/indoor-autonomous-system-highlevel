"""Adafruit motor controller for PCA9685 board and Exceed Magnet car

Author: Jared Jensen
"""

import Adafruit_PCA9685


class MotorController:
    def __init__(self, pulse_middle=390, pulse_diff=80, channel=1):
        """Controls the car's actuators.

        Keyword arguments:
        pulse_middle: The pulse needed to get the actuator to the middle
        pulse_diff: The maximum amount the actuator should deviate from the middle
            min pulse is pulse_middle - pulse_diff (go right)
            max pulse is pulse_middle + pulse_diff (go left)
        channel: The output channel for the actuator
        """

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pulse_middle = pulse_middle
        self.pulse_diff = pulse_diff
        self.channel = channel
        self.current_status = 0

        # Set up pwm
        self.pwm.set_pwm_freq(60)
        self.pwm.set_pwm(self.channel, 0, pulse_middle)

    def set_pwm(self, level):
        """Sets pulse width.

        Arguments:
        angle: a floating point number between -1 and 1.
            -1 is minimum
            1 is maximum
            -1 is full left
            0 is straight
            1 is full right
        """
        level = max(min(level, 1), -1)  # Make sure pulse is between 1 and -1
        self.current_status = level
        pulse_length = int(self.pulse_middle - (self.pulse_diff * level))
        self.pwm.set_pwm(self.channel, 0, pulse_length)


class ThrottleController(MotorController):
    def __init__(self, pulse_middle=390, pulse_diff=-80, channel=0):

        MotorController.__init__(self, pulse_middle, pulse_diff, channel)

        self.cruise_level = 0.6

    def set_throttle(self, level):
        """Sets the throttle level.

        Arguments:
        level: a floating point number between -1 and 1.
            -1 is reverse at max speed
            0 is stopped
            1 is forward at max speed
        """
        print("Setting throttle to ", level)
        self.set_pwm(level)

class SteeringController(MotorController):
    def __init__(self, pulse_middle=390, pulse_diff=80, channel=1):
        MotorController.__init__(self, pulse_middle, pulse_diff, channel)

    def turn(self, amount):
        """Turns the car.

        Arguments:
        amount: a floating point number between -1 and 1.
            -1 is left
            0 is straight
            1 is right
        """
        self.set_pwm(amount)
