#!/usr/bin/env python
"""
Main thread for running the state machine.

Author: Sagar Doshi
Date: 05/2018
"""

import rospy
import smach_ros

import unittest
import threading
import time

# Custom modules import
import sm_mach

if __name__ == "__main__":
    rospy.init_node('fezzik_state_machine')

    # Create the state machine
    sm = sm_mach.init_sm()

    # Create and start introspection server - automatically traverses sm's child
    # containers, so only need to add this to the top-level state machine
    sis = smach_ros.IntrospectionServer("Fezzik_Introspection_Server", sm, "/SM_TOP")
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()

    sis.stop()
