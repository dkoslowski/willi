#!/bin/env python3

# Below imports all necessary packages to make this Python Script run
import time
import board
from adafruit_motorkit import MotorKit

# Below initialises the variable kit to be our I2C Connected Adafruit Motor HAT. If stacking Multiple
# Adafruit HATs we can then explain in code exactly which I2C address we want focused on here.
kit = MotorKit(i2c = board.I2C())

#throttle = 1.0
#throttle = 0.5
throttle = 0.3333

#motor = kit.motor2
motor = kit.motor4

#kit.motor2.throttle = throttle
#kit.motor4.throttle = throttle

motor.throttle = throttle

time.sleep (4)

#kit.motor2.throttle = None
#kit.motor4.throttle = None

motor.throttle = None

