#!/usr/bin/env python
"""
This joy_publisher is for AURERO

Author: Roni Saputra
Date: -
"""
# ROS-related imports
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

import math
import os
import time
import signal
import sys
import xbox                     # Custom xbox module

SPEED_MULTIPLIER = -1          # Speed scaling factor
DIRECTION_MULTIPLIER = 0.5      # Direction scaling factor

# Protocol Data
HEADER      = 127       # Magic sync Header for the serial packet communication with the mbed
SPEED_CMD   = int(83)
MODE_CMD    = int(53)   # 73
RESET_CMD   = int(63)

joy         = xbox.Joystick()           # Initialize joystick

def cleanupOnExit():
    print("-- Cleaning up and quiting!")
    #control_sig.data = [SPEED_CMD, int(0), int(0), int(0)] 
    #pub_data.publish(control_sig)

    joy.close()
    # sys.exit(0)

def waitConnect():
    # Waiting for joystick connection
    print("-- Waiting for joystick connection")
    while not joy.connected():
        time.sleep(0.01)
    print("-- Joystick connected")

def executeControls():
    needInit = True                # check for initialization
    DriveMode = True                # True: Motor drive, False: Actuator drive

    while True:
        time.sleep(0.005)
        if not joy.connected():
            control_sig = Int32MultiArray()
            control_sig.data = [0, 0, 0]
            pub_data.publish(control_sig)
            print("-- Error! Joystick disconnected - waiting for connection")
            needInit = True
            time.sleep(0.1)

        else:
            print("Joystick Connected")  
            left_trig   = joy.leftTrigger()
            right_trig  = joy.rightTrigger()
            #print(left_trig, right_trig)

            y = -joy.leftY()
            
            if y<0:
                x = joy.leftX()
            else:
                x = -joy.leftX()
                    
            # speed = 100*(right_trig) - 100*(left_trig)
            speed = y*100
            speed = speed*SPEED_MULTIPLIER
            direction = x*100
            direction = direction * DIRECTION_MULTIPLIER

            # Initialise mbed
            if joy.Y() == 1:
                needInit = True               
        
            if speed > 0:
                speed, direction = int(speed), int(direction)
            else:
                speed, direction = int(speed), -int(direction)
            print("-- Speed: %d Direction: %d Drive mode: %d" % (speed, direction, DriveMode))
            cmd = SPEED_CMD

            if needInit:
                print("-- Initialisation")
                # Initialization: I-ASCII; Toggles between actuation and drive mode
                cmd = MODE_CMD 
                DriveMode = not DriveMode
                needInit = False
        
            # Soft reset of Mbed
            if joy.B() == 1:
                cmd = RESET_CMD 
                DriveMode = True
                control_sig = Int32MultiArray()

                control_sig.data = [0, 0, cmd]
                pub_data.publish(control_sig)
                time.sleep(0.05)

            
            '''
            if speed >= 0:
                speed = int(speed+9)
            else:
                speed = int(speed-9)

            if direction >= 0:
                direction = int(direction+9) 
            else:
                direction = int(direction-9)
            '''
            if right_trig>0.5:
                speed = speed*2
                direction = direction*2
            if left_trig>0.5:
                speed = speed*2
                direction = direction*4

            control_sig = Int32MultiArray()
            sig = Float32MultiArray()

            control_sig.data = [speed, direction, 0]
            sig.data = [float(speed), float(direction)]
            pub_data.publish(control_sig)
            pub.publish(sig)

'''
            control_sig = Int32MultiArray()
	        control_sig.data = [cmd, speed, direction, int(0)] # [forward velocity, angular velocity]
            pub_data.publish(control_sig)
'''
            

#ROS node initializations and subscription setup
rospy.init_node('aurero_control', anonymous = False)
#pub_data = rospy.Publisher("aurero_control_sig", Int32MultiArray, queue_size=1)
pub_data = rospy.Publisher('/resqbot/control_sig', Int32MultiArray, queue_size=1)
pub = rospy.Publisher('base_control_sig', Float32MultiArray)

if (__name__ == "__main__"):
    # Handle kill commands from terminal
    signal.signal(signal.SIGTERM, cleanupOnExit)

    # Waiting for joystick connection at the beginning
    #waitConnect()

    try:
        executeControls()
    except KeyboardInterrupt:
        cleanupOnExit()


