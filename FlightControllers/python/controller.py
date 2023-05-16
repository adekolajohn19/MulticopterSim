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
        # if  true it is right, false is left
        self.done= False
        self.startTime = 0
    

    def getLeftDemands(self, target, alt, vel):

        # Compute dzdt setpoint and error
        velError = (target - alt) - vel

        # Always compute throttle demand for altitude hold
        throttle = self.Kp * velError

        # Don't mess with roll,pitch, and yaw for this simple demo
        roll = 0
        pitch = 0
        yaw = 0.01

        return throttle, roll, pitch, yaw
    
    def getRightDemands(self, target, alt, vel):

        # Compute dzdt setpoint and error
        velError = (target - alt) - vel

        # Always compute throttle demand for altitude hold
        throttle = self.Kp * velError

        # Don't mess with roll,pitch, and yaw for this simple demo
        roll = 0
        pitch = 0
        yaw = 0.01

        return throttle, roll, pitch, yaw    