#!/bin/env python3

# Below imports all necessary packages to make this Python Script run
import time
from adafruit_motorkit import MotorKit

# Below initialises the variable kit to be our I2C Connected Adafruit Motor HAT. If stacking Multiple
# Adafruit HATs we can then explain in code exactly which I2C address we want focused on here.
kit = MotorKit(pwm_frequency = 50.0)

throttle = 0.3333


kit.motor2.throttle = throttle
kit.motor4.throttle = throttle


time.sleep (4)

kit.motor2.throttle = None
kit.motor4.throttle = None

