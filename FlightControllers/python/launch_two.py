#!/usr/bin/env python3
'''
Simple take-off-and-move-forward script

Copyright (C) 2021 Simon D. Levy

MIT License
'''

try:
    import cv2
except Exception:
    pass
from time import time
import numpy as np
import argparse
from argparse import ArgumentDefaultsHelpFormatter
import imutils
from controller import LaunchController
from multicopter_server import MulticopterServer
from mixers import PhantomMixer, IngenuityMixer
from debugging import debug


class LaunchCopter(MulticopterServer):

    def __init__(
            self,
            mixer,
            kp=1.0,
            ki=0.0,
            initial_target=5.0):

        MulticopterServer.__init__(self)

        self.mixer = mixer
        self.done=False
        self.time = 0
        self.target = initial_target
        cv2.setUseOptimized(onoff=True)
        # Create PID controller
        self.ctrl = LaunchController(kp, ki)

    def handleImage(self, image):
        try:
            
            image =  imutils.resize(image, width=600)
            # Convert the imageFrame in BGR(RGB color space) to HSV(hue-saturation-value) color space
            hsvFrame=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
       
        	# Set range for blue color and define mask
            blue_lower = np.array([0, 100, 100])
            blue_upper = np.array([10, 255, 255])
            blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
            blue_mask = cv2.erode(blue_mask, None, iterations=2)
            blue_mask = cv2.dilate(blue_mask, None, iterations=2)
            cnts = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None
            
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            #print(center)
            print(x)    
          
            cv2.circle(image, (int(x), int(y)), int(radius),
                            (0, 255, 255), 2)
            cv2.circle(image, center, 5, (0, 0, 255), -1)
           
            #EDIT:
            if x > 290 and  x < 310:
                print("middle")

            elif x > 1 and  x < 290:
                print("left")
            
            elif x > 310 and x < 600:
                print("right")
                

           

            # show the frame to our screen
            cv2.imshow("Image", image)
            key = cv2.waitKey(1) & 0xFF

           

        except Exception:
            debug('Failed')
            quit

    def getMotors(self, t, state, _stickDemands):

        # Track current time to share it with handleImage()
        self.time = t

        # Extract altitude from state.  Altitude is in NED coordinates, so we
        # negate it to use as input to PID controller.
        z = -state[MulticopterServer.STATE_Z]
        dzdt = -state[MulticopterServer.STATE_DZ]

        # Get demands U [throttle, roll, pitch, yaw] from PID controller,
        # ignoring stick demands
        if self.done==False:
            self.done=True
            self.ctrl.startTime= time()
        u = self.ctrl.getDemands(self.target, z, dzdt)

        # Use mixer to convert demands U into motor values Omega
        omega = self.mixer.getMotors(u)

        # Constrain motor values to [0,1]
        omega[omega > 1] = 1
        omega[omega < 0] = 0

        # Return motor values
        return omega


def main():

    parser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('--vehicle', required=False, default='Phantom',
                        help='Vehicle name')

    args = parser.parse_args()

    d = {'Phantom': PhantomMixer, 'Ingenuity': IngenuityMixer}

    if args.vehicle in d:
        copter = LaunchCopter(d[args.vehicle]())
        copter.start()

    else:
        debug('Unrecognized vehicle: %s' % args.vehicle)


main()