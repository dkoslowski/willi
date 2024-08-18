#! /usr/bin/env python3

import time
import willi

print("Calibrating willi's motors")

willi = willi.Willi()

try:
    print("Press ctrl+c to interrupt")

    while True:
        s = input('Enter the throttle value (-1.0 to 1.0): ')
        try:
            throttle = float(s)
            print('Set throttle to ', throttle)
            willi._set_throttle(throttle, throttle)
        except ValueError:
            print('Not a float: ', s)

except KeyboardInterrupt:
    print("\nInterrpted")

