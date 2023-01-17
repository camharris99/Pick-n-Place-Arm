"""!
The state machine that implements the logic.
"""
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import numpy as np
import rospy

class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoint_flag = False
        self.waypoints = [[0,0,0,0,0]]
        """  [-np.pi/2,       -0.5,      -0.3,            0.0,       0.0],
            [0.75*-np.pi/2,   0.5,      0.3,      0.0,       np.pi/2],
            [0.5*-np.pi/2,   -0.5,     -0.3,     np.pi / 2,     0.0],
            [0.25*-np.pi/2,   0.5,     0.3,     0.0,       np.pi/2],
            [0.0,             0.0,      0.0,         0.0,     0.0],
            [0.25*np.pi/2,   -0.5,      -0.3,      0.0,       np.pi/2],
            [0.5*np.pi/2,     0.5,     0.3,     np.pi / 2,     0.0],
            [0.75*np.pi/2,   -0.5,     -0.3,     0.0,       np.pi/2],
            [np.pi/2,         0.5,     0.3,      0.0,     0.0],
            [0.0,             0.0,     0.0,      0.0,     0.0]] """

    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "record_waypoint":
            self.record_waypoint()

        if self.next_state == "set_open_waypoint":
            self.set_open_waypoint()

        if self.next_state == "set_close_waypoint":
            self.set_close_waypoint()

        if self.next_state == "clear_waypoints":
            self.clear_waypoints()

        if self.next_state == "calibrate":
            self.calibrate()


    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        self.status_message = "State: Execute - Executing motion plan"
        self.next_state = "idle"

        #self.rxarm.set_moving_time(2.)
        #self.rxarm.set_accel_time(5.)

        for waypoint in self.waypoints:

            if np.linalg.norm(waypoint) == np.linalg.norm(np.array([[ 3, 1, 40, -9.5, 0]])):
                self.rxarm.open_gripper()

            elif np.linalg.norm(waypoint) == np.linalg.norm(np.array([[ 2.5, 82, 87.5, -4.5, 0]])):
                self.rxarm.close_gripper()

            else:
                self.rxarm.set_positions(waypoint)

            rospy.sleep(2.)

    def set_open_waypoint(self):
        """!
        @brief      open claw waypoint set
              Make sure you respect estop signal
        """
        self.status_message = "Sets waypoint that robot knows means open le gripper"
        self.next_state = "idle" 

        open_waypoint = np.array([[ 3, 1, 40, -9.5, 0]])

        self.waypoints.append(open_waypoint)               

    def set_close_waypoint(self):
        """!
        @brief      close claw waypoint set
              Make sure you respect estop signal
        """
        self.status_message = "Sets waypoint that robot  knows means close le gripper"
        self.next_state = "idle" 

        close_waypoint = np.array([[ 2.5, 82, 87.5, -4.5, 0]])

        self.waypoints.append(close_waypoint)   

    def clear_waypoints(self):
        """!
        @brief      clear waypoints
              Make sure you respect estop signal
        """
        self.status_message = "Waypoints have been reset"
        self.next_state = "idle" 

        self.waypoints = [[0,0,0,0,0]]

    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"

        """TODO Perform camera calibration routine here"""
        self.status_message = "Starting  - Click an Apriltag"
        # self.camera.new_click = False
        self.tags = [[0,0],[0,0],[0,0],[0,0]]
        for i in range(4):
            self.camera.new_click == False
            rospy.sleep(1)
            while(self.camera.new_click == False):
                #print("in loop")
                a = 0
            self.tags[i] = [self.camera.last_click[0],self.camera.last_click[1]]
            print(i)
            
        
        print("Clicked")
        print(self.tags)
        self.status_message = "Calibration - Completed Calibration"

        self.next_state = "idle"


    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        rospy.sleep(1)

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            rospy.sleep(5)
        self.next_state = "idle"

    def record_waypoint(self):
        """!
        @brief      Records the current joint angles of the manipulator arm
        """

        self.status_message = "State: Record Waypoint - Recording Waypoint"
        self.next_state = "idle"
        #print('record_waypoint has been called')
        #print(self.rxarm.get_positions())
        
        if self.waypoint_flag == False:
            self.waypoints[0] = self.rxarm.get_positions()
            self.waypoint_flag = True
        else:
            self.waypoints.append(self.rxarm.get_positions())

        #print(self.waypoints)

class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            rospy.sleep(0.05)