#!/usr/bin/python
"""!
Main GUI for Arm lab
"""
import os
script_path = os.path.dirname(os.path.realpath(__file__))

import argparse
import sys
import cv2
import math
import numpy as np
import rospy
import time
from functools import partial

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel,
                         QMainWindow, QCursor, QFileDialog)

from ui import Ui_MainWindow
import kinematics
from rxarm import RXArm, RXArmThread
from camera import Camera, VideoThread
from state_machine import StateMachine, StateMachineThread
""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi


class Gui(QMainWindow):
    """!
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """
    def __init__(self, parent=None, dh_config_file=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.MouseXYZ = np.zeros([3,1])
        self.GripFlag = True
        self.cobra = D2R*np.array([0., -8., 14.33, -63.81, 0.])
        """ Groups of ui commonents """
        self.joint_readouts = [
            self.ui.rdoutBaseJC,
            self.ui.rdoutShoulderJC,
            self.ui.rdoutElbowJC,
            self.ui.rdoutWristAJC,
            self.ui.rdoutWristRJC,
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutBase,
            self.ui.rdoutShoulder,
            self.ui.rdoutElbow,
            self.ui.rdoutWristA,
            self.ui.rdoutWristR,
        ]
        self.joint_sliders = [
            self.ui.sldrBase,
            self.ui.sldrShoulder,
            self.ui.sldrElbow,
            self.ui.sldrWristA,
            self.ui.sldrWristR,
        ]
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        self.next_state = "idle"

        """TODO Perform camera calibration routine here"""
        self.status_message = "Calibration - Completed Calibration"

        """Objects Using Other Classes"""
        self.camera = Camera()
        print("Creating rx arm...")
        if (dh_config_file is not None):
            self.rxarm = RXArm(dh_config_file=dh_config_file)
        else:
            self.rxarm = RXArm()
        print("Done creating rx arm instance.")
        self.sm = StateMachine(self.rxarm, self.camera)
        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        # Video
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse
        self.ui.videoDisplay.mousePressEvent = self.calibrateMousePress

        # Buttons
        # Handy lambda function falsethat can be used with Partial to only set the new state if the rxarm is initialized
        #nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state if self.rxarm.initialized else None)
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state)
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btn_init_arm.clicked.connect(self.initRxarm)
        self.ui.btn_torq_off.clicked.connect(
            lambda: self.rxarm.disable_torque())
        self.ui.btn_torq_on.clicked.connect(lambda: self.rxarm.enable_torque())
        self.ui.btn_sleep_arm.clicked.connect(lambda: self.rxarm.sleep())

        #User Buttons
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))
        self.ui.btnUser2.setText('Open Gripper')
        self.ui.btnUser2.clicked.connect(lambda: self.rxarm.open_gripper())
        self.ui.btnUser3.setText('Close Gripper')
        self.ui.btnUser3.clicked.connect(lambda: self.rxarm.close_gripper())
        self.ui.btnUser4.setText('Execute')
        self.ui.btnUser4.clicked.connect(partial(nxt_if_arm_init, 'execute'))
        self.ui.btnUser5.setText('Record Waypoint')
        self.ui.btnUser5.clicked.connect(partial(nxt_if_arm_init, 'record_waypoint'))
        self.ui.btnUser6.setText('Set Open Waypoint')
        self.ui.btnUser6.clicked.connect(partial(nxt_if_arm_init, 'set_open_waypoint'))
        self.ui.btnUser7.setText('Set Close Waypoint')
        self.ui.btnUser7.clicked.connect(partial(nxt_if_arm_init, 'set_close_waypoint'))
        self.ui.btnUser8.setText('Clear Waypoints Array')
        self.ui.btnUser8.clicked.connect(partial(nxt_if_arm_init, 'clear_waypoints'))
    
        # Slidersworld
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrMoveTime.valueChanged.connect(self.sliderChange)
        self.ui.sldrAccelTime.valueChanged.connect(self.sliderChange)
        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        self.ui.chk_autonomy.stateChanged.connect(self.autoControlChk)

        # Status
        self.ui.rdoutStatus.setText("Waiting for input")
        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)
        """Setup Threads"""

        # State machine
        self.StateMachineThread = StateMachineThread(self.sm)
        self.StateMachineThread.updateStatusMessage.connect(
            self.updateStatusMessage)
        self.StateMachineThread.start()
        self.VideoThread = VideoThread(self.camera)
        self.VideoThread.updateFrame.connect(self.setImage)
        self.VideoThread.start()
        self.ArmThread = RXArmThread(self.rxarm)
        self.ArmThread.updateJointReadout.connect(self.updateJointReadout)
        self.ArmThread.updateEndEffectorReadout.connect(
            self.updateEndEffectorReadout)
        self.ArmThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint * R2D)))

    ### TODO: output the rest of the orientation according to the convention chosen
    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f mm" % (pos[0])))
        self.ui.rdoutY.setText(str("%+.2f mm" % (pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f mm" % (pos[2])))
        self.ui.rdoutPhi.setText(str("%+.2f deg" % (R2D*pos[3])))
        self.ui.rdoutTheta.setText(str("%+.2f" % (R2D*pos[4])))
        self.ui.rdoutPsi.setText(str("%+.2f" % (R2D*pos[5])))

    @pyqtSlot(QImage, QImage, QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, tag_image, grid_image, contour_image):
        """!
        @brief      Display the images from the camera.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        if (self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if (self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if (self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(tag_image))
        if (self.ui.radioUsr2.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(grid_image))
        if (self.ui.radioUsr3.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(contour_image))
        

    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.rxarm.disable_torque()
        self.sm.set_next_state('estop')

    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutMoveTime.setText(
            str(self.ui.sldrMoveTime.value() / 10.0) + "s")
        self.ui.rdoutAccelTime.setText(
            str(self.ui.sldrAccelTime.value() / 20.0) + "s")
        self.rxarm.set_moving_time(self.ui.sldrMoveTime.value() / 10.0)
        self.rxarm.set_accel_time(self.ui.sldrAccelTime.value() / 20.0)

        # Do nothing if the rxarm is not initialized
        if self.rxarm.initialized:
            joint_positions = np.array(
                [sldr.value() * D2R for sldr in self.joint_sliders])
            # Only send the joints that the rxarm has
            self.rxarm.set_positions(joint_positions[0:self.rxarm.num_joints])

    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the rxarm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.rxarm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)

    def autoControlChk(self, state):
        """!
        @brief      Changes to autonomoy mode

                    Will only work if the rxarm is initialized.

        @param      state  State of the checkbox
        """
        
        if state == Qt.Checked and self.rxarm.initialized:
            # Enter autonomous mode
            self.sm.set_next_state("autonomy")
            self.sm.autoFlag = True
            
        elif self.rxarm.initialized == False:
            print("initialize the arm, dumbass")

        else:
            # Exit autonomous mode
            self.sm.set_next_state("idle")
            self.ui.chk_autonomy.setChecked(False)
            self.sm.autoFlag = False

    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """

        pt = mouse_event.pos()
        if self.camera.DepthFrameRaw.any() != 0:
            
            z = self.camera.DepthFrameRaw[pt.y()][pt.x()]
            intrinsic = self.camera.intrinsic_matrix
            extrinsic = self.camera.extrinsic_matrix
            
            if self.camera.homography.size != 0:
                #print(self.camera.homography)
                uv_trans = np.zeros([3,1])
                #print(np.array([[pt.x()],[pt.y()],[z]]))
                uv_hom = np.array([[pt.x()],[pt.y()],[1]])
                uv_trans = np.matmul( np.linalg.inv(self.camera.homography) , uv_hom )
                uv_trans[0,0] /= uv_trans[2,0]
                uv_trans[1,0] /= uv_trans[2,0]
                uv_trans[2,0] = 1
                #self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                 #                            (uv_trans[0,0], uv_trans[1,0], uv_trans[2,0]))
                #print(uv_trans)
                z = self.camera.DepthFrameRaw[int(uv_trans[1,0])][int(uv_trans[0,0])]
                #print(z)
                cam_coords = z*np.matmul(np.linalg.inv(intrinsic), uv_trans)
                #print(cam_coords)
                #cam_coords = z*np.matmul(np.linalg.inv(intrinsic), [pt.x(), pt.y(), 1]) 
                
                #H_inv = np.matmul( np.linalg.inv(self.camera.homography) , extrinsic )
                self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                                             (pt.x(), pt.y(), z))

            else:
                self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                                             (pt.x(), pt.y(), z))

                cam_coords = z*np.matmul(np.linalg.inv(intrinsic), [pt.x(), pt.y(), 1])
            #print(str(pt.x()) + ", " + str(pt.y()))
            H_inv =  extrinsic
         
            world_coords = np.matmul(H_inv, np.append(cam_coords,1))
            self.MouseXYZ[:,0] = np.reshape( np.array( [world_coords[0], world_coords[1], world_coords[2]] ), (3,) )
            #print(np.append(cam_coords,1    ))
            self.ui.rdoutMouseWorld.setText( "(%.0f, %.0f, %.0f)" % (world_coords[0], world_coords[1], world_coords[2]))


    def changeMoveSpeed(self, next_pose):
        """!
        @brief change speed of robot arm movement

        """
        
        next = next_pose
        curr = self.rxarm.get_positions()
        diff = next - curr
        weighted = np.multiply(diff,np.array([3.75,4,2,1.5,1.5]))
        norm = np.linalg.norm(weighted, ord=2)
        return norm/4


    def calibrateMousePress(self, mouse_event):
        """!
        @brief Record mouse click positions for calibration --> we actually are just using this for IK not calibration... could
               change the file name but we might need to change it elsewhere so be aware of that

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        """ Get mouse posiiton """
        pt = mouse_event.pos()
        pose = np.zeros([6,1])
        # left click!
        if mouse_event.button() == 1:
            
            pass

        # right click!    
        elif mouse_event.button() == 2:
            # setting x,y,z position

            

            pose[0:3,0] = np.reshape( self.MouseXYZ , (3,))
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

                solns, solnpsi = kinematics.IK_geometric(prepsi, pose)
                move = self.changeMoveSpeed(solns[1,:])
                self.rxarm.set_moving_time(move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(solns[1,:])
                rospy.sleep(1.5)
                self.rxarm.close_gripper()
                self.GripFlag = False

                # pre leaving pose --> back up and lift a bit
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
                #print(move)
                self.rxarm.set_moving_time(move)
                self.rxarm.set_accel_time(move/4)
                self.rxarm.set_positions(self.cobra)


            #### move = self.changeMoveSpeed(solns[1,:])
            #### self.rxarm.set_moving_time(move)
            #### self.rxarm.set_accel_time(move/4)
            #### solns[1,0] -= 2 * D2R #should account for this
            #### self.rxarm.set_positions(solns[1,:])
        self.camera.last_click[0] = pt.x()
        self.camera.last_click[1] = pt.y()
        self.camera.new_click = True
        # print(self.camera.last_click)

    def initRxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)
        self.ui.chk_autonomy.setChecked(False)
        self.rxarm.enable_torque()
        self.sm.set_next_state('initialize_rxarm')


### TODO: Add ability to parse POX config file as well
def main(args=None):
    """!
    @brief      Starts the GUI
    """
    app = QApplication(sys.argv)
    app_window = Gui(dh_config_file=args['dhconfig'])
    app_window.show()
    sys.exit(app.exec_())


# Run main if this file is being run directly
### TODO: Add ability to parse POX config file as well
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-c",
                    "--dhconfig",
                    required=False,
                    help="path to DH parameters csv file")
    main(args=vars(ap.parse_args()))
