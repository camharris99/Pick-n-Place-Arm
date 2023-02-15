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
        self.event_selection = ""
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
            while(self.event_selection == ""):
                print("waiting for task input")
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
        weighted = np.multiply(diff,np.array([3,3.5,2,.5,2]))
        norm = np.linalg.norm(weighted, ord=2)
        return norm/4

    def moveBlock(self, input_pose, height, prev_psi, block_angle=0, e5Flag=False):
        """!
        @brief move the block from one location to another

        @param input_pose The xyz phi theta psi coordinates of the desired End effector location
        @param The height of the block being moved
        @param The psi angle of the end effector during the previous moveBlock call
        @param block_angle Optional parameter that allows us to set theta_5 

        """
        pose = np.zeros([6,1])
        a = np.copy(input_pose[0])
        b = np.copy(input_pose[1])
        c = np.copy(input_pose[2])
        # pose[0:3,0] = np.reshape( input_pose , (3,))
        pose[0] = a
        pose[1] = b
        pose[2] = c
        # print(pose[2,0])
            #print(kinematics.IK_geometric(math.pi/4, pose))
            # # setting phi, theta, psi values -- keeping as zero for now b/c this shit no work!
            # # no change to pose because these values are already zero
            # # now we should call the inverse kinematics function to return the joint angles to reach the desired mouse position

        ##                                              ##
        ## THIS SECTION IS FOR WHEN THE GRIPPER IS OPEN ## 
        ##                                              ##

        if self.GripFlag == True: # i.e. gripper is open

            # arm pose after closing gripper on block
            leave_pose = np.copy(pose)
            leave_pose[0,0] = pose[0,0]*0.9
            leave_pose[1,0] = pose[1,0]*0.9
            leave_pose[2,0] = pose[2,0] + 75 # [mm]

            # arm pose upon approach before dropping off or picking up block
            pre_pose = np.copy(pose)
            pre_pose[2,0] = pose[2,0] + height # [mm]

            # if the block is not lined up with the grid, figure out the best ee oritentation for block pickup
            if block_angle != 0:

                presoln, prepsi = kinematics.IK_geometric(math.pi/4, pre_pose, self.GripFlag, block_angle)
            
            # block lined up with grid
            else:

                presoln, prepsi = kinematics.IK_geometric(math.pi/4, pre_pose, self.GripFlag)

            int_pose = np.zeros([1,5])
            int_pose = np.copy(presoln[1,:])
            int_pose[1] = -35.95*D2R
            int_pose[2] = 21.18*D2R
            int_pose[3] = -59.59*D2R
            int_pose[4] = 0.

            move = self.changeMoveSpeed(int_pose)
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(int_pose)
            rospy.sleep(1)


            # setting move speed for pre approach to block or drop off
            move = self.changeMoveSpeed(presoln[1,:])
            self.rxarm.set_moving_time(move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(presoln[1,:])
            rospy.sleep(2)

            # if the end effector is horizontal then make the x and y coordinate of the leaving pose the same as upon approach (i.e. only movement 
            # in the z direction when leaving)
            if prepsi == 0:
                leave_pose[0,0] = np.copy(pre_pose[0,0])
                leave_pose[1,0] = np.copy(pre_pose[1,0])
                
                # this accounts for the difference in the end effector z location between when the end effector is horizontal vs vertical
                #if prev_psi == math.pi/2:

                # i actually think this is just to make the gripper pick up blocks closer to the ee center when ee is horizontal
                #print(pose[2,0])

                # small block
                if height == 25:
                    pose[2,0] -= 16
                # large block
                elif height == 38: 
                    pose[2,0] -= 18 # [mm]
                #print(pose[2,0])

            # if the block is not lined up with the grid
            if block_angle != 0 and block_angle != 90:
                
                # movement to actually surround block with end effector --> if the block is not parallel with the grid
                solns, solnpsi = kinematics.IK_geometric(prepsi, pose, self.GripFlag, block_angle)
                #print(solns)
            else:
                # soln with block parallel to grid
                solns, solnpsi = kinematics.IK_geometric(prepsi, pose, self.GripFlag)

            move = self.changeMoveSpeed(solns[1,:])
            self.rxarm.set_moving_time(3*move)
            self.rxarm.set_accel_time(move/4)
            self.rxarm.set_positions(solns[1,:])
            rospy.sleep(1.5)
            self.rxarm.close_gripper()
            self.GripFlag = False

            # pre leaving pose --> back up and lift a bit
            # if the block is not parallel
            if block_angle != 0 and block_angle != 90:

                lsoln, leavepsi = kinematics.IK_geometric(solnpsi, leave_pose, self.GripFlag, block_angle)
                
            # if the block is parallel
            else:

                lsoln, leavepsi = kinematics.IK_geometric(solnpsi, leave_pose, self.GripFlag)

            if e5Flag == True:
                # if we are going to pick up the 12th block during event 5
                move = self.changeMoveSpeed(lsoln[1,:])
                self.rxarm.set_moving_time(6*move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(lsoln[1,:])
                rospy.sleep(1)

                int_pose = np.zeros([1,5])
                int_pose = np.copy(lsoln[1,:])
                int_pose[1] = -35.95*D2R
                int_pose[2] = 21.18*D2R
                int_pose[3] = -59.59*D2R
                int_pose[4] = 0.

                move = self.changeMoveSpeed(int_pose)
                self.rxarm.set_moving_time(6*move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(int_pose)
                rospy.sleep(1)

            else:
                # any other movement than the 12th block on event 5
                move = self.changeMoveSpeed(lsoln[1,:])
                self.rxarm.set_moving_time(move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(lsoln[1,:])
                rospy.sleep(1)

                int_pose = np.zeros([1,5])
                int_pose = np.copy(lsoln[1,:])
                int_pose[1] = -35.95*D2R
                int_pose[2] = 21.18*D2R
                int_pose[3] = -59.59*D2R
                int_pose[4] = 0.

                move = self.changeMoveSpeed(int_pose)
                self.rxarm.set_moving_time(move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(int_pose)
                rospy.sleep(1)

            # move = self.changeMoveSpeed(self.cobra)
            # self.rxarm.set_moving_time(move)
            # self.rxarm.set_accel_time(move/4)
            # self.rxarm.set_positions(self.cobra)
            # rospy.sleep(1)

        ##                                                ##
        ## THIS SECTION IS FOR WHEN THE GRIPPER IS CLOSED ## 
        ##                                                ##
        

        else:       # i.e. gripper is closed
        
            # pose to leave the block once it has been dropped of
            leave_pose = np.copy(pose)
            # leave_pose[0,0] *= 0.9
            # leave_pose[1,0] *= 0.9
            
            
            # pose to appraoch drop off before assuming drop-off position
            pre_pose = np.copy(pose)
            # adding offset to pre drop off position to ensure no bonks
            pre_pose[2,0] += 60 # [mm]

            # solving for solution to get approach position

            presoln, prepsi = kinematics.IK_geometric(math.pi/4, pre_pose, self.GripFlag)

            # offset to ensure the grabber doesn't knock over blocks when it leaves from stacking
            if prepsi == math.pi/2:
                leave_pose[2,0] += 75 # [mm]
            else: 
                leave_pose[2,0] += 100 # [mm]

            int_pose = np.zeros([1,5])
            int_pose = np.copy(presoln[1,:])
            int_pose[1] = -35.95*D2R
            int_pose[2] = 21.18*D2R
            int_pose[3] = -59.59*D2R
            int_pose[4] = 0.

            if e5Flag == True:
                # moving the twelthfh block
                move = self.changeMoveSpeed(int_pose)
                self.rxarm.set_moving_time(6*move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(int_pose)
                rospy.sleep(5)

                move = self.changeMoveSpeed(presoln[1,:])
                self.rxarm.set_moving_time(6*move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(presoln[1,:])
                rospy.sleep(5)

                # this accounts for the increase in height needed to place a block without crashing it into the board or a
                # target stack of blocks
                # vertical end effector orientation:
                if np.abs(prepsi) == math.pi/2:
                
                    pose[2,0] += height*.6
                
                # horizontal end effector orientation:
                else:
                    pose[2,0] += height*.55

                    if height == 25:
                        pose[2,0] -= 7

                # # horizontal to vertical end effector  orientatin change
                if prepsi == np.abs(math.pi/2) and prev_psi == 0.:  
                    pose[2,0] += 10 #[mm]

                solns, solnpsi = kinematics.IK_geometric(prepsi, pose, self.GripFlag)

                move = self.changeMoveSpeed(solns[1,:])
                self.rxarm.set_moving_time(8*move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(solns[1,:])
                rospy.sleep(5)
                self.rxarm.open_gripper()
                self.GripFlag = True
                print("z: " + str(pose[2,0]))
                # pre leaving pose --> back up and lift a bit
                lsoln, leavepsi = kinematics.IK_geometric(solnpsi, leave_pose, self.GripFlag)

                move = self.changeMoveSpeed(lsoln[1,:])
                self.rxarm.set_moving_time(6*move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(lsoln[1,:])
                rospy.sleep(5)

                int_pose = np.zeros([1,5])
                int_pose = np.copy(lsoln[1,:])
                int_pose[1] = -35.95*D2R
                int_pose[2] = 21.18*D2R
                int_pose[3] = -59.59*D2R
                int_pose[4] = 0.

                move = self.changeMoveSpeed(int_pose)
                self.rxarm.set_moving_time(6*move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(int_pose)
                rospy.sleep(5)

            else:
                # not movinv the twetlthth block
                move = self.changeMoveSpeed(int_pose)
                self.rxarm.set_moving_time(move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(int_pose)
                rospy.sleep(1)

                move = self.changeMoveSpeed(presoln[1,:])
                self.rxarm.set_moving_time(move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(presoln[1,:])
                rospy.sleep(2)

                # this accounts for the increase in height needed to place a block without crashing it into the board or a
                # target stack of blocks
                # vertical end effector orientation:
                if np.abs(prepsi) == math.pi/2:
                
                    pose[2,0] += height*.6
                
                # horizontal end effector orientation:
                else:
                    pose[2,0] += height*.55

                    if height == 25:
                        pose[2,0] -= 7

                # # horizontal to vertical end effector  orientatin change
                if prepsi == np.abs(math.pi/2) and prev_psi == 0.:  
                    pose[2,0] += 10 #[mm]

                solns, solnpsi = kinematics.IK_geometric(prepsi, pose, self.GripFlag)

                move = self.changeMoveSpeed(solns[1,:])
                self.rxarm.set_moving_time(3*move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(solns[1,:])
                rospy.sleep(2.5)
                self.rxarm.open_gripper()
                self.GripFlag = True
                print("z: " + str(pose[2,0]))
                # pre leaving pose --> back up and lift a bit
                lsoln, leavepsi = kinematics.IK_geometric(solnpsi, leave_pose, self.GripFlag)

                move = self.changeMoveSpeed(lsoln[1,:])
                self.rxarm.set_moving_time(move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(lsoln[1,:])
                rospy.sleep(1)

                int_pose = np.zeros([1,5])
                int_pose = np.copy(lsoln[1,:])
                int_pose[1] = -35.95*D2R
                int_pose[2] = 21.18*D2R
                int_pose[3] = -59.59*D2R
                int_pose[4] = 0.

                move = self.changeMoveSpeed(int_pose)
                self.rxarm.set_moving_time(move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(int_pose)
                rospy.sleep(1)

        return solnpsi

    def sweep_stack(self, input_pose):
        self.rxarm.close_gripper()
        self.GripFlag = False

        pre_swipe_sol, prepsi = kinematics.IK_geometric(math.pi/4, input_pose, self.GripFlag)

        post_swipe_sol = np.copy(pre_swipe_sol)

        if (input_pose[0] >= 0):
            pre_swipe_sol[1,0] += 15 * D2R
            post_swipe_sol[1,0] -= 10 * D2R
        else:
            pre_swipe_sol[1,0] -= 15 * D2R
            post_swipe_sol[1,0] += 10 * D2R

        print("pre swipe location")

        move = self.changeMoveSpeed(pre_swipe_sol[1,:])
        self.rxarm.set_moving_time(move)
        self.rxarm.set_accel_time(move/4)
        self.rxarm.set_positions(pre_swipe_sol[1,:])
        rospy.sleep(1)

        print("post swipe")

        move = 4*self.changeMoveSpeed(post_swipe_sol[1,:])
        self.rxarm.set_moving_time(move)
        self.rxarm.set_accel_time(move/4)
        self.rxarm.set_positions(post_swipe_sol[1,:])
        rospy.sleep(1)

        leave_sol = np.copy(post_swipe_sol)
        leave_sol[1,1] -= 15 * D2R

        move = self.changeMoveSpeed(leave_sol[1,:])
        self.rxarm.set_moving_time(move)
        self.rxarm.set_accel_time(move/4)
        self.rxarm.set_positions(leave_sol[1,:])
        rospy.sleep(1)

        self.rxarm.open_gripper()
        self.GripFlag = True

        pass



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
        prev_psi = 0.
        block_coordsXYZ = list(self.camera.block_coords)

        while (len(block_coordsXYZ) < self.camera.num_blocks):
            block_coordsXYZ = list(self.camera.block_coords)
        
        block_coordsXYZ = list(self.camera.block_coords)
        """
        can start placing logic based on "self.event_selection" here
        """

        def sort_by_norm(val):
                    return np.linalg.norm(val.XYZ)
        def sort_by_stack(val):
                    return val.stacked
        def sort_by_color(val):
                    return val.color_index

        if (self.event_selection == "event 1"):     

            block_coordsXYZ.sort(key=sort_by_norm)
            large_drop_pt_world = np.array([-375, -100, 0])
            small_drop_pt_world = np.array([375, -100, 0])

            for elem in block_coordsXYZ:
                x = elem.XYZ[0]
                y = elem.XYZ[1]
                z = elem.XYZ[2]
                if elem.shape == "large":
                    elem.XYZ[2] -= 7
                angle = elem.angle
                shape = elem.shape
                prev_psi = self.moveBlock(elem.XYZ, elem.height, prev_psi, angle)
                rospy.sleep(0.5)

                if (elem.shape == "large"):
                    prev_psi = self.moveBlock(large_drop_pt_world, elem.height, prev_psi)
                    if (large_drop_pt_world[0] >= -175):
                        large_drop_pt_world[2] += elem.height
                    else:
                        large_drop_pt_world[0] += 60
                elif (elem.shape == "small"):
                    prev_psi = self.moveBlock(small_drop_pt_world, elem.height, prev_psi)
                    if (small_drop_pt_world[0] <= 175):
                        small_drop_pt_world[2] -= elem.height
                    else:
                        small_drop_pt_world[0] -= 40

            # now that all is done, lets deal with any remaining stacks
            remaining_blocks = 1000
            while (remaining_blocks > 0):
                print("starting stack detection")
                self.camera.blockDetector(True)
                block_coordsXYZ = list(self.camera.block_coords)

                while (len(block_coordsXYZ) < self.camera.num_blocks):
                    block_coordsXYZ = list(self.camera.block_coords)

                for i in range(len(block_coordsXYZ)-1, -1, -1):
                    x = block_coordsXYZ[i].XYZ[0]
                    y = block_coordsXYZ[i].XYZ[1]
                    z = block_coordsXYZ[i].XYZ[2]

                    if (y < 0):
                        del block_coordsXYZ[i]
                        if (len(block_coordsXYZ) == 0):
                            # print("no stacks left!")
                            remaining_blocks = 0
                        # print("block coord deleted: ", i)
                        # print("current block_coord length: ", len(block_coordsXYZ))
                        continue

                    block_coordsXYZ[i].XYZ[2] -= elem.height/4
                    angle = block_coordsXYZ[i].angle
                    shape = block_coordsXYZ[i].shape
                    prev_psi = self.moveBlock(block_coordsXYZ[i].XYZ, block_coordsXYZ[i].height, prev_psi, angle)
                    rospy.sleep(0.5)

                    if (shape == "large"):
                        prev_psi = self.moveBlock(large_drop_pt_world, block_coordsXYZ[i].height, prev_psi)
                        if (large_drop_pt_world[0] >= -175):
                            large_drop_pt_world[2] += block_coordsXYZ[i].height
                        else:
                            large_drop_pt_world[0] += 60
                    elif (shape == "small"):
                        prev_psi = self.moveBlock(small_drop_pt_world, block_coordsXYZ[i].height, prev_psi)
                        if (small_drop_pt_world[0] <= 175):
                            small_drop_pt_world[2] -= block_coordsXYZ[i].height
                        else:
                            small_drop_pt_world[0] -= 40
                    
                    else:
                        print("stacks found!")

                pass
            
        if (self.event_selection == "event 2"):
            
            ## NOTE : THIS EVENT 2 CODE IS NOT FUNCTIONAL. DONT EXPECT IT DO TO DO THE INTENDED THING FOR EVEENT 2 YET  

            h1 = 0
            h2 = 0
            h3 = 0
            staq_1 = np.array([-300, -100, h1])
            staq_2 = np.array([250, -100, h2])
            staq_3 = np.array([-150, -100, h3])            

            # swiper_height = 60
            # for elem in block_coordsXYZ:
            #     if (elem.stacked == False):
            #         continue
            #     print("stack found!")
            #     x = elem.XYZ[0]
            #     y = elem.XYZ[1]
            #     z = np.copy(elem.XYZ[2])
            #     elem.XYZ[2] = swiper_height
            #     self.sweep_stack(elem.XYZ)

            self.camera.blockDetector(True)

            block_coordsXYZ = list(self.camera.block_coords)

            while (len(block_coordsXYZ) < self.camera.num_blocks):
                block_coordsXYZ = list(self.camera.block_coords)

            block_coordsXYZ.sort(key=sort_by_norm)


            stack_counter = 0
            for elem in block_coordsXYZ:

                x = elem.XYZ[0]
                y = elem.XYZ[1]
                z = elem.XYZ[2]
                print("going for: ", elem.color, " ", elem.shape, " block!")
                # picking up the block
                if elem.shape == "large":
                    elem.XYZ[2] -= 7
                # elif elem.shape == "small":
                #     elem.XYZ[2] += 2
                angle = elem.angle
                shape = elem.shape
                prev_psi = self.moveBlock(elem.XYZ, elem.height, prev_psi, angle)
                rospy.sleep(0.5)

                # deciding where to put the block
                if (stack_counter<= 2):
                    
                    print("height of stack 1: " + str(h1))
                    prev_psi = self.moveBlock(staq_1, elem.height, prev_psi)
                    h1 += elem.height
                    staq_1[2] = h1

                elif (stack_counter<= 5):
                    
                    print("height of stack 2: " + str(h2))
                    prev_psi = self.moveBlock(staq_2, elem.height, prev_psi)
                    h2 += elem.height
                    staq_2[2] = h2

                else:
                    
                    print("height of stack 3: " + str(h3))
                    prev_psi = self.moveBlock(staq_3, elem.height, prev_psi)
                    h3 += elem.height
                    staq_3[2] = h3

                stack_counter+= 1
            # now that all is done, lets deal with any remaining stacks
            remaining_blocks = 1000
            while (remaining_blocks > 0):
                print("starting stack detection")
                self.camera.blockDetector(True)
                block_coordsXYZ = list(self.camera.block_coords)

                while (len(block_coordsXYZ) < self.camera.num_blocks):
                    block_coordsXYZ = list(self.camera.block_coords)

                for i in range(len(block_coordsXYZ)-1, -1, -1):
                    x = block_coordsXYZ[i].XYZ[0]
                    y = block_coordsXYZ[i].XYZ[1]
                    z = block_coordsXYZ[i].XYZ[2]

                    if (y < 0):
                        del block_coordsXYZ[i]
                        if (len(block_coordsXYZ) == 0):
                            # print("no stacks left!")
                            remaining_blocks = 0
                        # print("block coord deleted: ", i)
                        # print("current block_coord length: ", len(block_coordsXYZ))
                        continue

                    block_coordsXYZ[i].XYZ[2] -= elem.height/3.5
                    angle = block_coordsXYZ[i].angle
                    shape = block_coordsXYZ[i].shape
                    prev_psi = self.moveBlock(block_coordsXYZ[i].XYZ, block_coordsXYZ[i].height, prev_psi, angle)
                    rospy.sleep(0.5)

                    if (stack_counter<= 2):
                    
                        print("height of stack 1: " + str(h1))
                        prev_psi = self.moveBlock(staq_1, elem.height, prev_psi)
                        h1 += elem.height
                        staq_1[2] = h1

                    elif (stack_counter<= 5):
                    
                        print("height of stack 2: " + str(h2))
                        prev_psi = self.moveBlock(staq_2, elem.height, prev_psi)
                        h2 += elem.height
                        staq_2[2] = h2

                    else:
                    
                        print("height of stack 3: " + str(h3))
                        prev_psi = self.moveBlock(staq_3, elem.height, prev_psi)
                        h3 += elem.height
                        staq_3[2] = h3

                    stack_counter+= 1

            #     pass
        
        if (self.event_selection == "event 3"):          
            """
            idea to build this event:
                start with just level 2... 
                    start with finding any "likely stacks" and knock them over
                    find all blocks and sort them into two lists (large and small blocks)
                    sort the lists into ROYGBV order
                    use same pre-determined line locations as in event 1
                once level 2 is figured out...
                    detect if there are any blocks in the line we want to place at
                    if there are, move them to some location where we can pick them up
                    re do the image to make sure we deal with all the blocks
            """

            block_coordsXYZ.sort(key=sort_by_stack)
            swiper_height = 50

            red_large_drop_pt = np.array([-360, -100, 0])
            red_small_drop_pt = np.array([350, -100, 0])

            orange_large_drop_pt = np.copy(red_large_drop_pt)
            orange_large_drop_pt[0] += 48
            orange_small_drop_pt = np.copy(red_small_drop_pt)
            orange_small_drop_pt[0] -= 35

            yellow_large_drop_pt = np.copy(orange_large_drop_pt)
            yellow_large_drop_pt[0] += 48
            yellow_small_drop_pt = np.copy(orange_small_drop_pt)
            yellow_small_drop_pt[0] -= 35

            green_large_drop_pt = np.copy(yellow_large_drop_pt)
            green_large_drop_pt[0] += 48
            green_small_drop_pt = np.copy(yellow_small_drop_pt)
            green_small_drop_pt[0] -= 35

            blue_large_drop_pt = np.copy(green_large_drop_pt)
            blue_large_drop_pt[0] += 48
            blue_small_drop_pt = np.copy(green_small_drop_pt)
            blue_small_drop_pt[0] -= 35

            purple_large_drop_pt = np.copy(blue_large_drop_pt)
            purple_large_drop_pt[0] += 48
            purple_small_drop_pt = np.copy(blue_small_drop_pt)
            purple_small_drop_pt[0] -= 35

            # knock down all stacked
            # for elem in block_coordsXYZ:
            #     if (elem.stacked == False):
            #         continue
            #     x = elem.XYZ[0]
            #     y = elem.XYZ[1]
            #     z = np.copy(elem.XYZ[2])
            #     elem.XYZ[2] = swiper_height
            #     self.sweep_stack(elem.XYZ)
            
            # removing blocks from negative y section
            x_spot = -200
            for elem in block_coordsXYZ:

                # if the block is in the negative y plane
                
                if elem.XYZ[1] < 0:

                    prev_psi = self.moveBlock(elem.XYZ, elem.height, prev_psi, elem.angle)
                    rospy.sleep(.5)
                    prev_psi = self.moveBlock(np.array([[x_spot],[250],[100]]), elem.height, math.pi/4)
                    rospy.sleep(.5)
                    x_spot += 100


            self.camera.blockDetector(True)
            block_coordsXYZ = list(self.camera.block_coords)
            print("num blocks: ", self.camera.num_blocks)

            while (len(block_coordsXYZ) < self.camera.num_blocks):
                block_coordsXYZ = list(self.camera.block_coords)

            block_coordsXYZ.sort(key=sort_by_norm)

            for elem in block_coordsXYZ:
                x = elem.XYZ[0]
                y = elem.XYZ[1]
                z = elem.XYZ[2]

                if elem.shape == "large":
                    elem.XYZ[2] -= 7
                #elif elem.shape == "small":
                #    elem.XYZ[2] -= 

                angle = elem.angle
                shape = elem.shape
                prev_psi = self.moveBlock(elem.XYZ, elem.height, prev_psi, angle)
                rospy.sleep(0.5)

                if (elem.shape == "large"):
                    if (elem.color == "red"):
                        prev_psi = self.moveBlock(red_large_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "orange"):
                        prev_psi = self.moveBlock(orange_large_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "yellow"):
                        prev_psi = self.moveBlock(yellow_large_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "green"):
                        prev_psi = self.moveBlock(green_large_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "blue"):
                        prev_psi = self.moveBlock(blue_large_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "purple"):
                        prev_psi = self.moveBlock(purple_large_drop_pt, elem.height, prev_psi)
                    
                elif (elem.shape == "small"):
                    if (elem.color == "red"):
                        prev_psi = self.moveBlock(red_small_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "orange"):
                        prev_psi = self.moveBlock(orange_small_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "yellow"):
                        prev_psi = self.moveBlock(yellow_small_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "green"):
                        prev_psi = self.moveBlock(green_small_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "blue"):
                        prev_psi = self.moveBlock(blue_small_drop_pt, elem.height, prev_psi)
                    elif (elem.color == "purple"):
                        prev_psi = self.moveBlock(purple_small_drop_pt, elem.height, prev_psi)

            # now that all is done, lets deal with any remaining stacks
            remaining_blocks = 1000
            while (remaining_blocks > 0):
                print("starting stack detection")
                self.camera.blockDetector(True)
                block_coordsXYZ = list(self.camera.block_coords)

                while (len(block_coordsXYZ) < self.camera.num_blocks):
                    block_coordsXYZ = list(self.camera.block_coords)

                for i in range(len(block_coordsXYZ)-1, -1, -1):
                    x = block_coordsXYZ[i].XYZ[0]
                    y = block_coordsXYZ[i].XYZ[1]
                    z = block_coordsXYZ[i].XYZ[2]

                    if (y < 0):
                        del block_coordsXYZ[i]
                        if (len(block_coordsXYZ) == 0):
                            # print("no stacks left!")
                            remaining_blocks = 0
                        # print("block coord deleted: ", i)
                        # print("current block_coord length: ", len(block_coordsXYZ))
                        continue
                    if block_coordsXYZ[i].shape == "large":
                        block_coordsXYZ[i].XYZ[2] -= 7

                    angle = block_coordsXYZ[i].angle
                    shape = block_coordsXYZ[i].shape
                    prev_psi = self.moveBlock(block_coordsXYZ[i].XYZ, block_coordsXYZ[i].height, prev_psi, angle)
                    rospy.sleep(0.5)

                    if (block_coordsXYZ[i].shape == "large"):
                        if (block_coordsXYZ[i].color == "red"):
                            prev_psi = self.moveBlock(red_large_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "orange"):
                            prev_psi = self.moveBlock(orange_large_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "yellow"):
                            prev_psi = self.moveBlock(yellow_large_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "green"):
                            prev_psi = self.moveBlock(green_large_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "blue"):
                            prev_psi = self.moveBlock(blue_large_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "purple"):
                            prev_psi = self.moveBlock(purple_large_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        
                    elif (block_coordsXYZ[i].shape == "small"):
                        if (block_coordsXYZ[i].color == "red"):
                            prev_psi = self.moveBlock(red_small_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "orange"):
                            prev_psi = self.moveBlock(orange_small_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "yellow"):
                            prev_psi = self.moveBlock(yellow_small_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "green"):
                            prev_psi = self.moveBlock(green_small_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "blue"):
                            prev_psi = self.moveBlock(blue_small_drop_pt, block_coordsXYZ[i].height, prev_psi)
                        elif (block_coordsXYZ[i].color == "purple"):
                            prev_psi = self.moveBlock(purple_small_drop_pt, block_coordsXYZ[i].height, prev_psi)

                        else:
                            print("stacks found!")


        if (self.event_selection == "event 4"):
            block_coordsXYZ.sort(key=sort_by_stack)
            swiper_height = 50

            large_drop_pt_world = np.array([-200, -100, 0])
            small_drop_pt_world = np.array([200, -100, 0])

            # knock down all stacked
            for elem in block_coordsXYZ:
                if (elem.stacked == False):
                    continue
                x = elem.XYZ[0]
                y = elem.XYZ[1]
                z = np.copy(elem.XYZ[2])
                elem.XYZ[2] = swiper_height
                self.sweep_stack(elem.XYZ)

            rospy.sleep(1)

            self.camera.blockDetector(True)
            block_coordsXYZ = list(self.camera.block_coords)

            while (len(block_coordsXYZ) < self.camera.num_blocks):
                block_coordsXYZ = list(self.camera.block_coords)

            block_coordsXYZ.sort(key=sort_by_color)

            for elem in block_coordsXYZ:
                x = elem.XYZ[0]
                y = elem.XYZ[1]
                z = elem.XYZ[2]

                if elem.shape == "large":
                    elem.XYZ[2] -= 7
                #elif elem.shape == "small":
                #    elem.XYZ[2] -= 

                angle = elem.angle
                shape = elem.shape
                prev_psi = self.moveBlock(elem.XYZ, elem.height, prev_psi, angle)
                rospy.sleep(0.5)

                if (elem.shape == "large"):
                    prev_psi = self.moveBlock(large_drop_pt_world, elem.height, prev_psi)
                    large_drop_pt_world[2] += elem.height
                elif (elem.shape == "small"):
                    prev_psi = self.moveBlock(small_drop_pt_world, elem.height, prev_psi)
                    small_drop_pt_world[2] += elem.height


        if (self.event_selection == "event 5"):
            def sort_by_norm(val):
            
                return np.linalg.norm(val.XYZ)
            
            block_coordsXYZ.sort(key=sort_by_norm)

            #blk_cnt = 0
            staq_height = 0.
            for elem in block_coordsXYZ:
                #print(elem)
                #print(" ")
                x = elem.XYZ[0]
                y = elem.XYZ[1]
                z = elem.XYZ[2]
                print("z in autonomy: ", z)
                if elem.shape == "large":
                    elem.XYZ[2] -= 4
                angle = elem.angle
                shape = elem.shape
                if x < -450 or x > 450 or y < -150 or y > 450 or z < -5:
                    print("passed point")
                    pass
                else:

                    # this is for the first 11 blocks
                    if x > 0:
                        print("starting movement")
                        #print(prev_psi)
                        prev_psi = self.moveBlock(elem.XYZ, elem.height, prev_psi, angle)
                        rospy.sleep(0.5)
                        
                        drop_pt_world = np.array([[0],[225],[staq_height]]) 
                            
                        prev_psi = self.moveBlock(np.array([drop_pt_world[0],drop_pt_world[1],staq_height]), elem.height, prev_psi)      
                        
                        rospy.sleep(0.5)
                        staq_height += elem.height
                        #blk_cnt += 1
                    
                    else: 
                        continue

            # rescanning for negative x coordinate blocks ( should be 3 for now )
            self.camera.blockDetector(True)
            block_coordsXYZ = list(self.camera.block_coords)
            while (len(block_coordsXYZ) < self.camera.num_blocks):
                block_coordsXYZ = list(self.camera.block_coords)
            block_coordsXYZ.sort(key=sort_by_norm)

            count = 0
            height = 0
            for elem in block_coordsXYZ:

                # ignore the tall stack we have created
                if elem.XYZ[0] > -50:
                    continue
                # the blocks we are interested in:
                else:
                    if count < 2: 
                        prev_psi = self.moveBlock(elem.XYZ, elem.height, prev_psi, angle)
                        rospy.sleep(0.5)
                        drop_point = np.array([[block_coordsXYZ[2].XYZ[0]], [block_coordsXYZ[2].XYZ[1]], [height]])
                        # attempting to make the arm place the first two blocks on top of the third block which should be the farthest away
                        prev_psi = self.moveBlock(drop_point, elem.height, prev_psi)      
                        rospy.sleep(0.5)

                        count += 1
                        height += elem.height
                    else:
                        # going to the third blocks location
                        prev_psi = self.moveBlock(elem.XYZ, elem.height, prev_psi, angle, True)
                        rospy.sleep(0.5)


                        prev_psi = self.moveBlock(np.array([drop_pt_world[0],drop_pt_world[1],staq_height]), elem.height, prev_psi, True)  
                        rospy.sleep(0.5)


                            
   
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
