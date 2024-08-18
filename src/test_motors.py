#! /usr/bin/env python3

import time
import willi

# maximal speed 0,617427676 m/s

print("Testing willi's motors")

willi = willi.Willi()

try:
    print("Press ctrl+c to interrupt")

    for i in range(-10, 11):
        throttle = i / 10
        print("throttle:", throttle)
        willi._set_throttle(throttle, throttle)
        time.sleep(2)
    willi.stop()

    time.sleep(2)

    for i in range(-10, 11):
        speed = i / 10
        print("speed m/s:", speed)
        willi._set_speed(speed, speed)
        time.sleep(2)
    willi.stop()

    print("Finished")

except KeyboardInterrupt:
    print("\nInterrpted")

