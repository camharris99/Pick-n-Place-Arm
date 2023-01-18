"""!
The state machine that implements the logic.
"""
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import numpy as np
import rospy
import cv2
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
            [0.25*-np.pi/2,self.status_message = "Starting  - Click an Apriltag"   0.5,     0.3,     0.0,       np.pi/2],
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

    def recover_homogenous_transform_pnp(self, image_points, world_points, K, D):
        '''
        Use SolvePnP to find the rigidbody transform representing the camera pose in
        world coordinates (not working)
        '''

        # there is an error in this function when i call solvePnP :(
        # not sure how to fix it 
        distCoeffs = D
        [_, R_exp, t] = cv2.solvePnP(world_points,
                                 image_points,
                                 K,
                                 distCoeffs,
                                 flags=cv2.SOLVEPNP_ITERATIVE)
        R, _ = cv2.Rodrigues(R_exp)
        return np.row_stack((np.column_stack((R, t)), (0, 0, 0, 1)))




    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"

        """TODO Perform camera calibration routine here"""
        
        self.tags_uvd = np.array([[0,0,0],[0,0,0],
                                  [0,0,0],[0,0,0],
                                  [0,0,0],[0,0,0],
                                  [0,0,0],[0,0,0],
                                  [0,0,0],[0,0,0],
                                  [0,0,0],[0,0,0]])
        
        
        # get the xyz coords of the mouse location by inspection because they are known?
        # i think these must be known because we can't calculate them without the extrinsic matrix
        for i in range(12):
            self.camera.new_click = False
            # status messages are bunged ask for help bc they are not designed well
            # how to send out updateStatusMessage mid-state?
            self.status_message = "Starting  - Click Apriltag #" + str(i)
                
            
            while(self.camera.new_click == False):
               
                #could potentially replase this with "pass"
                pass
            z = self.camera.DepthFrameRaw[self.camera.last_click[1]][self.camera.last_click[0]]
            self.tags_uvd[i] = [self.camera.last_click[0],self.camera.last_click[1],z]
            

        points_uv = np.delete(self.tags_uvd,-1,axis=1)
        #print(points_uv)
        depth_camera = np.transpose(np.delete(self.tags_uvd, (0,1), axis=1))
        #print(depth_camera)

        # these are factory calibration settings
        D = np.array([0.13974332809448242, -0.45853713154792786, -0.0008287496748380363, 0.00018046400509774685, 0.40496668219566345])
        
        K = np.array([[900.543212890625, 0.0, 655.990478515625], 
                      [0.0, 900.89501953125, 353.4480285644531], 
                      [0.0, 0.0, 1.0]])
        Kinv = np.linalg.inv(K)

        points_world = np.array([[-450, 425, 0], [450,425,0],[-450,275,0],
                               [-250,275,0],[250,275,0],[450,275,0],
                               [-450,-25,0],[-250,-25,0],[250,-25,0],
                               [450,-25,0],[-450,-125,0],[450,-125,0]])
        
        points_ones = np.ones(depth_camera.size)

        points_camera = np.transpose(depth_camera*np.dot(Kinv,np.transpose(np.column_stack((points_uv,points_ones)))))


        #OpenCV SolvePNP calculate extrinsic matrix A
        print(points_uv.astype(np.float64))
        A_pnp = self.recover_homogenous_transform_pnp(points_world.astype(np.float32), points_uv.astype(np.float64),
                                                      K.astype(np.float64),D.astype(np.float64)) # 
        points_transformed_pnp = np.dot(np.linalg.inv(A_pnp), np.transpose(np.column_stack((points_camera, points_ones))))
        
        world_points = np.transpose(np.column_stack((points_world, points_ones)))

        print("\nWorld Points: \n")
        print(np.transpose(np.column_stack((points_world, points_ones))))


        print("\nSolvePnP: \n")
        print(A_pnp)
        print(points_transformed_pnp.astype(int))

        print("Clicked")
        #print(self.tags)
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
