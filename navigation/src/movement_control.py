#!/usr/bin/env python
"""
Send control commands to mobile base.

Author: Petar Kormushev
Date: 04/18
"""

import math
import os
import time
import serial
import sys

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Empty

HEADER      = 127       # Magic sync Header for the serial packet communication with the mbed
CMD         = 83        # Predetermined command for the serial communication with the mbed
RESET_CMD   = 63        # Predetermined command for reseting the mbed
INIT_CMD    = 53        # Take mbed out of time-out mode into normal mode

# Serial port data
PORT_NAME = "/dev/ttyUSB0"
# BAUD_RATE = int(230400)
BAUD_RATE = int(115200)
serial_port = serial.Serial()


def packData(velocity, direction, lights, cmd=None):
    cmd = CMD if cmd is None else cmd
    data = bytearray()
    data.append(HEADER)
    data.append(cmd)
    data.append(velocity & 0xff)
    data.append(direction & 0xff)
    data.append(lights & 0xff)

    return data


def sendToMbed(msg):
    """
    @ brief Send commands to the mbed. Note that msg has data of type float but
    the mobile base can understand only integers
    """
    if not serial_port.is_open:
        return
    
    v = int(msg.data[0]) # velocity
    w = int(msg.data[1]) # angular velocity
    try:
        lights = int(msg.data[2])
    except:
        lights = int(0)

    data = packData(v, w, lights)

    serial_port.write(data)
    #print(v, w, lights)


def cleanupShutdown():
    """
    @brief Triggered on shutdown in order to cleanup and then exit
    """
    print("-- Cleaning up and quiting!")

    # Unregister subscriber
    sub.unregister()

    serial_port.write(packData(int(0), int(0), int(0)))
    print("-- Mobile base stopped")

    serial_port.close()
    sys.exit()


def resetSerial():
    print("-- Resetting serial connection to mbed")
    serial_port.write(packData(int(0), int(0), int(0)))
    
    if serial_port.is_open:
      serial_port.reset_output_buffer()
      serial_port.close()
      time.sleep(2)
    initSerial()
    print("-- Serial connection to mbed reset")


def initSerial():
    if not serial_port.is_open:
        serial_port.baudrate = BAUD_RATE
        serial_port.port     = PORT_NAME
        serial_port.open()


def resetMbed(msg):
    print("Resetting mbed")
    data = packData(0, 0, 0, cmd=RESET_CMD)
    serial_port.write(data)
    print("mbed reset")
    resetSerial()


def resetMbedTimeout(msg):
    rospy.loginfo("Taking mbed out of timeout mode.")
    serial_port.write(packData(0,0,0,INIT_CMD)) # take mbed out of time-out


if __name__ == '__main__':
    # initialize the serial port connection
    initSerial()

    rospy.init_node('base_motion_control', anonymous = False)
    sub = rospy.Subscriber('base_control_sig', Float32MultiArray, sendToMbed, queue_size=1)
    sub = rospy.Subscriber('reset_mbed', Empty, resetMbed, queue_size=1)
    sub_reset_mbed_timeout = rospy.Subscriber('reset_mbed_timeout', Empty, resetMbedTimeout, queue_size=1)
    # Set cleanup function to be called on shutdown
    rospy.on_shutdown(cleanupShutdown)
    resetMbedTimeout(None)
    rospy.spin()
