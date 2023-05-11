'''
Simple altitude-hold and fly-forward

Copyright (C) 2019 Simon D. Levy

MIT License
'''

from time import time

class LaunchController(object):

    def __init__(self, Kp, windupMax=10):

        # Constants
        self.Kp = Kp
        self.windupMax = windupMax
        self.startTime = time()

    def getDemands(self, target, alt, vel):

        # Compute dzdt setpoint and error
        velError = (target - alt) - vel

        # Always compute throttle demand for altitude hold
        throttle = self.Kp * velError

        # Don't mess with roll,pitch, and yaw for this simple demo
        roll = 0
        #pitch =0
        pitch = -.01 if time() - self.startTime < 10 else 0
        yaw = 0

        return throttle, roll, pitch, yaw