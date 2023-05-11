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

import numpy as np
import argparse
from argparse import ArgumentDefaultsHelpFormatter

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

        self.time = 0
        self.target = initial_target

        # Create PID controller
        self.ctrl = LaunchController(kp, ki)

    def handleImage(self, image):
        try:
            """
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 100, 200)
            # Find the contours in the edge image
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the bounding box of the largest contour
            (x, y, w, h) = cv2.boundingRect(largest_contour)

            # Draw the bounding box on the frame
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # Get the center of the bounding box
            center = (x + w // 2, y + h // 2)

            # Calculate the distance between the center of the bounding box and the center of the camera
            distance = np.sqrt((center[0] - w // 2) ** 2 + (center[1] - h // 2) ** 2)

            # Display the distance
            print("The distance between the drone and the object is {} pixels".format(distance))            
            cv2.imshow('Edge Detection', image)

            cv2.waitKey(1)

            nonzero = np.nonzero(edges)[0]
"""
            # Convert the imageFrame in BGR(RGB color space) to HSV(hue-saturation-value) color space
            hsvFrame=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
       
        	# Set range for blue color and define mask
            blue_lower = np.array([0, 100, 100])
            blue_upper = np.array([10, 255, 255])
            blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

            # Morphological Transform, Dilation  for each color and bitwise_and operator  between imageFrame 
            # and mask determines to detect only that particular color
            kernel = np.ones((5, 5), "uint8")

            # Creating contour to track blue color
            contours, hierarchy = cv2.findContours(blue_mask,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
            
            # Sort the blobs by size, largest to smallest
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            # Get the biggest 3 blobs
            biggest_blobs = contours[:3]
            
         

            
            for pic, contour in enumerate(biggest_blobs):
                area = cv2.contourArea(contour)
                #print(area)

                x, y, w, h = cv2.boundingRect(contour)
                image = cv2.rectangle(image, (x, y),
                                            (x + w, y + h),
                                            (255, 0, 0), 2)	
                cv2.putText(image, "Blue", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1.0, (255, 0, 0))
            
                    
            # Program Termination
            cv2.imshow("Multiple Color Detection in Real-TIme", image)
            cv2.waitKey(1) 
            nonzero = np.nonzero(image)[0]
            # Ignore image for first five seconds
            if len(nonzero) > 0 and np.mean(nonzero) > 390 and self.time > 5:
                self.target = 30

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
