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
        self.startTime = float(0)
        self.okay=False

    def getLeftDemands(self, target, alt, vel):

        # Compute dzdt setpoint and error
        velError = (target - alt) - vel

        # Always compute throttle demand for altitude hold
        if time() - self.startTime>23:
            self.okay= True
        throttle = self.Kp * velError if time() - self.startTime < 21.95 else 0

        # Don't mess with roll,pitch, and yaw for this simple demo
        roll = 0
        pitch = 0
        yaw= -0.0001
        print(int(time()-self.startTime))
        if time() - self.startTime < 10:
            yaw = 0.0001
        elif time() - self.startTime > 12:
            pitch= -.0001
            yaw= 0
        return throttle, roll, pitch, yaw    
    
    def getRightDemands(self, target, alt, vel):

        # Compute dzdt setpoint and error
        velError = (target - alt) - vel

        # Always compute throttle demand for altitude hold
        if time() - self.startTime>23:
            self.okay= True
        throttle = self.Kp * velError if time() - self.startTime < 21.95 else 0

        # Don't mess with roll,pitch, and yaw for this simple demo
        roll = 0
        pitch = 0
        yaw= 0.0001
        print(int(time()-self.startTime))
        if time() - self.startTime < 10:
            yaw = -0.0001
        elif time() - self.startTime > 12:
            pitch= .0001
            yaw= 0
        return throttle, roll, pitch, yaw    