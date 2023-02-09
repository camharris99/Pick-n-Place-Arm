"""!
The state machine that implements the logic.
"""
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import numpy as np
import rospy
import cv2
import kinematics
import math

D2R = np.pi / 180.0
R2D = 180.0 / np.pi

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
        #self.MouseXYZ = np.zeros([3,1])
        self.GripFlag = True
        self.cobra = D2R*np.array([0., -8., 14.33, -63.81, 0.])
        self.autoFlag = False
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

        if self.next_state == "autonomy":
            self.autonomy()


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
                                 useExtrinsicGuess = False,
                                 flags=cv2.SOLVEPNP_ITERATIVE)
        R, _ = cv2.Rodrigues(R_exp)
        return np.row_stack((np.column_stack((R, t)), (0, 0, 0, 1)))

    def changeMoveSpeed(self, next_pose):
        """!
        @brief change speed of robot arm movement

        """
        
        next = next_pose
        curr = self.rxarm.get_positions()
        diff = next - curr
        weighted = np.multiply(diff,np.array([3.75,3.5,2,1.5,1.5]))
        norm = np.linalg.norm(weighted, ord=2)
        return norm/4

    def moveBlock(self, input_pose, block_angle=0):
        pose = np.zeros([6,1])
        pose[0:3,0] = np.reshape( input_pose , (3,))
            #print(kinematics.IK_geometric(math.pi/4, pose))
            # # setting phi, theta, psi values -- keeping as zero for now b/c this shit no work!
            # # no change to pose because these values are already zero
            # # now we should call the inverse kinematics function to return the joint angles to reach the desired mouse position
            
        if self.GripFlag == True: # i.e. gripper is open

            leave_pose = np.copy(pose)
            leave_pose[0,0] *= 0.9
            leave_pose[1,0] *= 0.9
            leave_pose[2,0] += 75 # [mm]

            pre_pose = np.copy(pose)
            pre_pose[2,0] += 60 # [mm]

            if block_angle != 0:

                presoln, prepsi = kinematics.IK_geometric(math.pi/4, pre_pose, block_angle)
                #print(presoln)
            
            else:
                presoln, prepsi = kinematics.IK_geometric(math.pi/4, pre_pose)

            move = self.changeMoveSpeed(presoln[1,:])
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(presoln[1,:])
            rospy.sleep(2)

            if prepsi == 0:
                leave_pose[0,0] *= 1.1
                leave_pose[1,0] *= 1.1
                pose[2,0] -= 30 # [mm]
            if block_angle != 0:

                solns, solnpsi = kinematics.IK_geometric(prepsi, pose,block_angle)
                #print(solns)
            else:

                solns, solnpsi = kinematics.IK_geometric(prepsi, pose)

            move = self.changeMoveSpeed(solns[1,:])
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(solns[1,:])
            rospy.sleep(1.5)
            self.rxarm.close_gripper()
            self.GripFlag = False

            # pre leaving pose --> back up and lift a bit
            if block_angle != 0:

                lsoln, leavepsi = kinematics.IK_geometric(solnpsi, leave_pose, block_angle)
                #print(lsoln)
            else:
                
                lsoln, leavepsi = kinematics.IK_geometric(solnpsi, leave_pose)
                
            move = self.changeMoveSpeed(lsoln[1,:])
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(lsoln[1,:])
            rospy.sleep(1)

            move = self.changeMoveSpeed(self.cobra)
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            
            self.rxarm.set_positions(self.cobra)
            rospy.sleep(1)

        else:       # i.e. gripper is closed
        
            leave_pose = np.copy(pose)
            leave_pose[0,0] *= 0.9
            leave_pose[1,0] *= 0.9
            leave_pose[2,0] += 75 # [mm]
            
            pre_pose = np.copy(pose)
            pre_pose[2,0] += 60 # [mm]

            presoln, prepsi = kinematics.IK_geometric(math.pi/4, pre_pose)
            move = self.changeMoveSpeed(presoln[1,:])
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(presoln[1,:])
            rospy.sleep(2)

            if prepsi == math.pi/2:
                pose[2,0] += 30
            else:
                pose[2,0] += 10
            solns, solnpsi = kinematics.IK_geometric(prepsi, pose)
            move = self.changeMoveSpeed(solns[1,:])
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(solns[1,:])
            rospy.sleep(2.5)
            self.rxarm.open_gripper()
            self.GripFlag = True

            # pre leaving pose --> back up and lift a bit
            lsoln, leavepsi = kinematics.IK_geometric(solnpsi, leave_pose)

            move = self.changeMoveSpeed(lsoln[1,:])
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(lsoln[1,:])
            rospy.sleep(1)

            move = self.changeMoveSpeed(self.cobra)
            rospy.sleep(1)
            #print(move)
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(self.cobra)


    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"

        """TODO Perform camera calibration routine here"""
        
        self.tags_uvd = np.zeros((7,3))
        
        # using april tag locations
        tags = np.zeros((7,3))
        #print(tags)
        for detection in self.camera.tag_detections.detections:
            # extracting xyz coordinates from the apriltag data packet --> this data is X_c,Y_c,Z_c
            self.tags_uvd[detection.id[0]-1,0] = detection.pose.pose.pose.position.x / detection.pose.pose.pose.position.z
            self.tags_uvd[detection.id[0]-1,1] = detection.pose.pose.pose.position.y / detection.pose.pose.pose.position.z
            self.tags_uvd[detection.id[0]-1,2] = 1
            # print()
        #print(self.tags_uvd)

        # THIS IS THE MANAUAL CLICK-AND-COLLECT MODE
        # get the xyz coords of the mouse location by inspection because they are known?
        # i think these must be known because we can't calculate them without the extrinsic matrix
        # for i in range(12):
        #     self.camera.new_click = False
        #     # status messages are bunged ask for help bc they are not designed well
        #     # how to send out updateStatusMessage mid-state?
        #     self.status_message = "Starting  - Click Apriltag #" + str(i)
                
            
        #     while(self.camera.new_click == False):
               
        #         #could potentially replase this with "pass"
        #         pass
        #     z = self.camera.DepthFrameRaw[self.camera.last_click[1]][self.camera.last_click[0]]
        #     self.tags_uvd[i] = [self.camera.last_click[0],self.camera.last_click[1],z]
        

        # these are factory calibration settings
        D = np.array([0.13974332809448242, -0.45853713154792786, -0.0008287496748380363, 0.00018046400509774685, 0.40496668219566345])

        K = np.array([[900.543212890625, 0.0, 655.990478515625], 
                      [0.0, 900.89501953125, 353.4480285644531], 
                      [0.0, 0.0, 1.0]])
                      
        Kinv = np.linalg.inv(K)

        # points_world = np.array([[-250, -25, 0], [250,-25,0],[250,275,0],
        #                        [-250,275,0],[475,-100,154],[-375,400,242],
        #                        [75,200,61], [-475,-50,154]])
        points_world = np.array([[-250, -25, 0], [250,-25,0],[250,275,0],
                               [-250,275,0],[475,-100,154],[-375,400,154],
                               [-475,-50,242]])
        #calculating the u, v, 1 pixel coordinate representation, structure is n x 3 matrix!!
        uvd_coords = np.transpose(np.matmul(K, np.transpose(self.tags_uvd)))
        #print(uvd_coords)
        # getting just uv coords
        points_uv = np.delete(uvd_coords,-1,axis=1)
        #print(points_uv)
        depth_camera = np.transpose(np.delete(uvd_coords, (0,1), axis=1))
        #print(depth_camera)
        
        points_ones = np.ones(depth_camera.size)
        
        # this is used later on for making sure stuff is calculated correctly
        #points_camera = np.transpose(depth_camera*np.dot(Kinv,np.transpose(np.column_stack((points_uv,points_ones)))))
        points_camera = self.tags_uvd
        
        #print(points_camera)
        #OpenCV SolvePNP calculate extrinsic matrix A
        # inv(A_pnp) is the extrinsic rotation matrix!!!
        A_pnp = self.recover_homogenous_transform_pnp(points_uv.astype(np.float64), points_world.astype(np.float64),
                                                      K,D) # 
        # used for making sure calculations are correct later on
        #print("invA")
        # print("camera points")
        # print(np.transpose(np.column_stack((points_camera, points_ones))))
        points_transformed_pnp = np.matmul(np.linalg.inv(A_pnp), np.transpose(np.column_stack((points_camera, points_ones))))
        #calculating world points for comparision
        world_points = np.transpose(np.column_stack((points_world, points_ones)))

        # print("\nWorld Points: \n")
        # print(world_points)

        self.camera.extrinsic_matrix = np.linalg.inv(A_pnp)
        # self.camera.extrinsic_matrix = (A_pnp)

        src_pts = points_uv[0:4,:]

        x_cen = 600
        y_cen = 350
        x_max = x_cen + 250
        x_min = x_cen - 250
        y_min = y_cen - 125
        y_max = y_cen + 175
        scale = 1.025
        pt1 = scale*np.array([x_min , y_max])
        pt2 = scale*np.array([x_max , y_max])
        pt3 = scale*np.array([x_max, y_min])
        pt4 = scale*np.array([x_min, y_min])
        dest_pts = np.array([pt1, pt2, pt3, pt4])
        # print("src:")
        # print(src_pts)
        # print("dest:")
        # print(dest_pts)
        H = cv2.findHomography(src_pts, dest_pts)[0]
        self.camera.homography = H
        # print("\nSolvePnP: \n")
        # print("Rotation Matrix:")
        # print(A_pnp)
        # print("Calculated world coords:")
        # print(points_transformed_pnp.astype(int))

        #print("Clicked")
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

    

    def autonomy(self):
        """!
        @brief logic to implement autonomous functionality of arm
        """
        self.status_message = "State: Autonomy"
        self.next_state = "idle"

        print("test pre block coords")
        print("there are: ", self.camera.num_blocks, " blocks!")
        pre_block_coordsXYZ = self.camera.block_coords.copy()
        print(np.shape(pre_block_coordsXYZ)[1])
        print(pre_block_coordsXYZ)

        while (np.shape(pre_block_coordsXYZ)[1] < self.camera.num_blocks + 1):
            pre_block_coordsXYZ = self.camera.block_coords.copy()
        
        print(pre_block_coordsXYZ)
        print(np.shape(pre_block_coordsXYZ)[1])
        print(self.camera.num_blocks)
        print("in armtonomy!")
        pre_block_coordsXYZ = self.camera.block_coords.copy()
        #print(pre_block_coordsXYZ)
        # this removes the first column of zeros that is necessary to not make numpy unhappy when hstacking stuff
        block_coordsXYZ = np.transpose(pre_block_coordsXYZ[:,1:np.shape(pre_block_coordsXYZ)[1]])
        print(block_coordsXYZ)
        for elem in block_coordsXYZ:
            #print(elem)
            #print(" ")
            x = elem[0]
            y = elem[1]
            z = elem[2]
            angle = elem[3]
            if x < -450 or x > 450 or y < -150 or y > 450 or z < -5:
                print("passed point")
                pass
            else:
                print("starting movement")
                self.moveBlock(elem[0:3], angle)
                rospy.sleep(0.5)
                
                # ADDING SOME STUFF TO POTENTIALLY IMPROVE AUTONOMY
                # Get depth reading at the desired placement x-y position
                # CHECK THE VALUE OF z BEFORE RUNNING AUTONOMY - MIGHT NEED TO SUBTRACT 1000

                intrinsic = self.camera.intrinsic_matrix
                extrinsic = self.camera.extrinsic_matrix
            
                if self.camera.homography.size != 0:
                #print(self.camera.homography)
                    uv_trans = np.zeros([3,1])
                #print(np.array([[pt.x()],[pt.y()],[z]]))
                    # CAN CUSTOMIZE THIS TO PICK ANY POINT ON THE BOARD OR CALCULATE A POINT TO PLACE THE BLOCK
                    
                    # pixel coordinates in homography --> they are in this form arbitarily, we could pick pre-homography pixel coords to get
                    # depth reading from... i just did it because that way i can pick points from the calibrated video display
                    uv_hom = np.array([[820],[284],[1]])

                    # undoing hommography transform
                    uv_trans = np.matmul( np.linalg.inv(self.camera.homography) , uv_hom )
                    
                    # normalizing x and y pixel locations
                    uv_trans[0,0] /= uv_trans[2,0]
                    uv_trans[1,0] /= uv_trans[2,0]
                    uv_trans[2,0] = 1
               
                    z = self.camera.DepthFrameRaw[int(uv_trans[1,0])][int(uv_trans[0,0])]
                #print(z)
                    cam_coords = z*np.matmul(np.linalg.inv(intrinsic), uv_trans)
                    #print(z)
                             
                    world_coords = np.matmul(extrinsic, np.append(cam_coords,1))
                    
                    # the depth reading in world coordinates of the specific point of interest for block placement
                    z_w = world_coords[2]
                    #print(z_w)
                self.moveBlock(np.array([200,225,z_w+7.5]))
                rospy.sleep(0.5)
                #self.moveBlock(np.array([0,425,0]))
        # end check to determine if the autonomy button is still pressed
        #if self.autoFlag == True:
        #    self.autonomy()
        #else:
        #    pass
   
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
