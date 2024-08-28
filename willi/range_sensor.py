#! /usr/bin/env python3

import lgpio
import time
import math

class RangeSensor:
    '''An HC-SR04 Ultrasonic Range Sensor

    Methods
    -------
    get_distance()
        Return the distance in meters seen by the sensor

    '''

    def __init__ (self, pinTrigger=17, pinEcho=18, speed_of_sound = 343.0):

        self._pinTrigger = pinTrigger
        self._pinEcho = pinEcho

        # Configure GPIO
        self._handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self._handle, self._pinTrigger)
        lgpio.gpio_claim_input(self._handle, self._pinEcho)


        self.speed_of_sound = speed_of_sound

        # these come from the sensor datasheet and are used in creating
        # a ROS Range message.
        # https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
        self.min_range = 0.030
        self.max_range = 4.000
        self.angle = 15.0 * math.pi / 180.0  #degrees to radians

        # pre-compute some static values
        self._min_travel_time = 2.0 * self.min_range / self.speed_of_sound
        self._max_travel_time = 2.0 * self.max_range / self.speed_of_sound

    def __del__(self):
        lgpio.gpiochip_close(self._handle)

    def get_distance(self):

        #  Trigger the measuring with a 10us pulse
        lgpio.gpio_write(self._handle, self._pinTrigger, 1)
        time.sleep (0.00001)
        lgpio.gpio_write(self._handle, self._pinTrigger, 0)
 
        initTime    = time.time()
        startTime   = initTime
        arrivalTime = initTime
    
        # wait for the message send
        while lgpio.gpio_read(self._handle, self._pinEcho) == 0:
            if startTime - initTime > 15.0:
                raise Exception('Timeout: No response from range sensor')
            startTime = time.time()
    
        # wait for the message arrival
        while lgpio.gpio_read(self._handle, self._pinEcho) == 1:
            arrivalTime = time.time()
            if arrivalTime - startTime > self._max_travel_time:
                # no iceberg ahead, all clear
                arrivalTime = startTime
                break

        return ((arrivalTime - startTime) * self.speed_of_sound) / 2


def main(args=None):

    # Measure the distance every second to test the module
    try:
        print("Press ctrl+c to interrupt")
        sensor = RangeSensor()
        while True:
            print('Distance measured: ', sensor.get_distance())
            time.sleep(1)

    except Exception as e:
        print(e)
    except KeyboardInterrupt:
        print('\nBye!')

if __name__ == '__main__':
    main()