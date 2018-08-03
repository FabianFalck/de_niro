#!/usr/bin/env python
"""
Contains information on state transitions. State transitions can be changed in the
'transitions = {TRANSITION: LISTENING}' lines below.

Author: Sagar Doshi
Date: 05/2018
"""

import smach

# Custom modules import
import sm_states

INIT_ROBOT          = 'INIT_ROBOT'
IDLING              = 'IDLING'
LISTENING           = 'LISTENING'
REMEMBERING_USER    = 'REMEMBERING_USER'
NAVIGATING          = 'NAVIGATING'
DETECTING_OBJECT    = 'DETECTING_OBJECT'
GRASPING            = 'GRASPING'
RETURNING           = 'RETURNING'
SEEKING_USER        = 'SEEKING_USER'
OFFERING_OBJECT     = 'OFFERING_OBJECT'

from sm_states import TRANSITION

### NOTE !!!
# Adding new state:
#   - Add a corresponding transition signal in the constants above
#   - Add the transition in MASTER_STATE transitions dictionary
#   - Add it to the initialisation conditions in MasterState in sm_states.py


def init_sm():
    """
    @brief: initialize the state machine
    @return - smach.StateMachine: the state machine
    """
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes = [TRANSITION])


    # Open the container and initialise it
    with sm:
        smach.StateMachine.add( INIT_ROBOT, sm_states.InitRobot(),
                                transitions = {TRANSITION: IDLING}
        )

        smach.StateMachine.add( IDLING, sm_states.Idling(),
                                transitions = {TRANSITION: LISTENING}
        )

        smach.StateMachine.add( LISTENING, sm_states.Listening(),
                                transitions = {TRANSITION: REMEMBERING_USER}
        )

        smach.StateMachine.add( REMEMBERING_USER, sm_states.RememberingUser(),
                                transitions = {TRANSITION: NAVIGATING}
        )

        smach.StateMachine.add( NAVIGATING, sm_states.Navigating(),
                                transitions = {TRANSITION: GRASPING}
        )

        smach.StateMachine.add( GRASPING, sm_states.Grasping(),
                                transitions = {TRANSITION: RETURNING}
        )

        smach.StateMachine.add( RETURNING, sm_states.Returning(),
                                transitions = {TRANSITION: SEEKING_USER}
        )

        smach.StateMachine.add( SEEKING_USER, sm_states.SeekingUser(),
                                transitions = {TRANSITION: OFFERING_OBJECT}
        )

        # Change to LISTENING to loop without break
        smach.StateMachine.add( OFFERING_OBJECT, sm_states.OfferingObject(),
                                transitions = {TRANSITION: IDLING}
        )

        return sm
