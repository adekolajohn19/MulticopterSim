'''
Simple altitude-hold and fly-forward

Copyright (C) 2019 Simon D. Levy

MIT License
'''

from time import time

class TunnelController(object):

    def __init__(self, Kp, windupMax=10):

        # Constants
        self.Kp = Kp
        self.windupMax = windupMax
        self.done= False
        self.startTime = 0
    def fromLeftDemands(self, target, alt, vel):
        # Compute dzdt setpoint and error
        velError = (0 - alt) - vel

        # Always compute throttle demand for altitude hold
        throttle =self.Kp * velError if time() - self.startTime < 3 else 0

        # Don't mess with roll,pitch, and yaw for this simple demo
        roll = 0
        pitch = 0

        if time() - self.startTime < .3:
            yaw = -5
        else:
            yaw = 0
            exit(0)
            throttle, roll, pitch, yaw = self.getDemands(target, alt, vel)
        

        return throttle, roll, pitch, yaw    
    
    def fromRightDemands(self, target, alt, vel):
        # Compute dzdt setpoint and error
        velError = (target - alt) - vel

        # Always compute throttle demand for altitude hold
        throttle = self.Kp * velError

        # Don't mess with roll,pitch, and yaw for this simple demo
        roll = 0
        pitch = 0
        yaw = 0.01

    def getDemands(self, target, alt, vel):

        # Compute dzdt setpoint and error
        velError = (target - alt) - vel

        # Always compute throttle demand for altitude hold
        throttle =self.Kp * velError if time() - self.startTime < 6 else 0
        # Don't mess with roll,pitch, and yaw for this simple demo
        roll = 0
        #pitch =0
        
        
        if time() - self.startTime < 2.4:
            pitch = -.01 
        elif 6 > time() - self.startTime > 2.41 :
            pitch = .01
        else:
            pitch = 0
        yaw = 0
        
    
        return throttle, roll, pitch, yaw
        