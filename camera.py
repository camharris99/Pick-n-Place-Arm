"""!
Class to represent the camera.
"""

import cv2
import time
import math
import numpy as np
from PyQt4.QtGui import QImage
from PyQt4.QtCore import QThread, pyqtSignal, QTimer
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from apriltag_ros.msg import *
from cv_bridge import CvBridge, CvBridgeError


class Camera():
    """!
    @brief      This class describes a camera.
    """
    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.VideoFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.GridFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.ContourFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720, 1280)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.array([[]])

        # mouse clicks & calibration variables
        self.cameraCalibrated = False
        self.intrinsic_matrix = np.array([[903.5076,   0,  649.9839],
                            [0,  907.2036,  333.0393],
                            [0,  0,   0.9744]])
        self.extrinsic_matrix = self.extrinsic_calc()
        self.last_click = np.array([0, 0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)
        self.grid_x_points = np.arange(-450, 500, 50)
        self.grid_y_points = np.arange(-175, 525, 50)
        self.xgrid_coord, self.ygrid_coord = np.meshgrid(self.grid_x_points, self.grid_y_points)
        self.grid_points = np.array(np.meshgrid(self.grid_x_points, self.grid_y_points)).T.reshape(-1,2)

        self.tag_detections = np.array([])
        self.tag_locations = [[-250, -25], [250, -25], [250, 275], [-250, 275],[-275, 150], [275, 150]]
        """ block info """
        self.block_contours = np.array([])
        self.block_detections = np.array([])
        self.homography = np.array([])
    def extrinsic_calc(self):
        cam_angle = 13
        t = 180-cam_angle
        tt = 1.75
        ttt = -1
        Tyb = np.array([[1,0,0,0],
           [0,1,0,350],
           [0,0,1,0],
           [0,0,0,1]])
   
        Tzc = np.array([[1,0,0,0],
           [0,1,0,0],
           [0,0,1,990],
           [0,0,0,1]])
       
        Rxt = np.array([[1,0,0,0],
           [0,math.cos(np.radians(t)), -1*math.sin(np.radians(t)),0],
           [0,math.sin(np.radians(t)), math.cos(np.radians(t)),0],
           [0,0,0,1]])

        Rztt = np.array([[math.cos(np.radians(tt)),-1*math.sin(np.radians(tt)),0,0],
           [math.sin(np.radians(tt)), math.cos(np.radians(tt)),0,0],
           [0,0,1,0],
           [0,0,0,1]])

        Ryttt = np.array([[math.cos(np.radians(ttt)),0,math.sin(np.radians(ttt)),0],
           [0,1,0,0],
           [-1*math.sin(np.radians(ttt)),0, math.cos(np.radians(ttt)),0],
           [0,0,0,1]])
 
        H = np.matmul(Tyb,Tzc)
        H = np.matmul(H,Rxt)
        H = np.matmul(H,Rztt)
        H = np.matmul(H,Ryttt)
 
        return(H)

    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame, self.block_contours, -1,
                         (255, 0, 255), 3)

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[..., 0] = self.DepthFrameRaw >> 1
        self.DepthFrameHSV[..., 1] = 0xFF
        self.DepthFrameHSV[..., 2] = 0x9F
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,
                                          cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png", cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",
                                        0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (1280, 720))
            #print(self.homography.size())
            if (self.homography.size !=0):
                #print("Applying warp")
                frame = cv2.warpPerspective(frame,self.homography,(frame.shape[1], frame.shape[0]))

            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtGridFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.GridFrame, (1280, 720))
            if (self.homography.size !=0):
                #print("Applying warp")
                frame = cv2.warpPerspective(frame,self.homography,(frame.shape[1], frame.shape[0]))

            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None
        
    def convertQtContourFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.ContourFrame, (1280, 720))
            if (self.homography.size !=0):
                #print("Applying warp")
                frame = cv2.warpPerspective(frame,self.homography,(frame.shape[1], frame.shape[0]))

            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None


    def convertQtDepthFrame(self):
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            img = QImage(self.DepthFrameRGB, self.DepthFrameRGB.shape[1],
                         self.DepthFrameRGB.shape[0], QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (1280, 720))
            if (self.homography.size !=0):
                #print("Applying warp")
                frame = cv2.warpPerspective(frame,self.homography,(frame.shape[1], frame.shape[0]))

            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

        @param      coord1  Points in coordinate frame 1
        @param      coord2  Points in coordinate frame 2

        @return     Affine transform between coordinates.
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        print(cv2.getAffineTransform(pts1, pts2))
        return cv2.getAffineTransform(pts1, pts2)

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

                    TODO: use this to load in any calibration files you need to

        @param      file  The file
        """
        pass

    def mask_and_contour(self, image, color):
        copy = image.copy()
        if (color == "red"):
            mask1 = cv2.inRange(copy, (174,50,50), (182, 255, 255))
            mask2 = cv2.inRange(copy, (0,50,50), (3, 255, 255))
            mask = mask1 + mask2
            contour_color = (175,255,255)
        elif (color == "orange"):
            mask = cv2.inRange(copy, (4,140,149), (12, 246, 218))
            contour_color = (8,250,250)
        elif (color == "yellow"):
            mask = cv2.inRange(copy, (20,135,160), (27, 255, 255))
            contour_color = (23,200,250)
        elif (color == "green"):
            mask = cv2.inRange(copy, (60,50,70), (90, 255, 255))
            contour_color = (75,255,255)
        elif (color == "blue"):
            mask = cv2.inRange(copy, (100,100,90), (107, 255, 255))
            contour_color = (102,255,255)
        elif (color == "purple"):
            mask = cv2.inRange(copy, (109,43,56), (135, 255, 255))
            contour_color = (115,120,255)

        # print(np.shape(copy))
        # print(np.shape(mask))
        copy[np.where(mask==0)] = [0,0,0]
        # print(np.shape(copy))
        copy = cv2.medianBlur(copy,3)
        copy = cv2.morphologyEx(copy, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
        copy = cv2.morphologyEx(copy, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8))

        _, copy = cv2.threshold(copy, 70,255, cv2.THRESH_BINARY)
        gray_copy = cv2.cvtColor(copy, cv2.COLOR_HSV2RGB)
        gray_copy = cv2.cvtColor(gray_copy, cv2.COLOR_RGB2GRAY)
        _, contours, _ = cv2.findContours(gray_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i in range(len(contours)):
            cv2.drawContours(image, contours, i, contour_color, 3)

        return image

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        img_hsv = cv2.cvtColor(self.VideoFrame.copy(), cv2.COLOR_RGB2HSV)
        contoured_image = self.mask_and_contour(img_hsv, "red")
        contoured_image = self.mask_and_contour(img_hsv, "green")
        contoured_image = self.mask_and_contour(img_hsv, "blue")
        contoured_image = self.mask_and_contour(img_hsv, "purple")
        contoured_image = self.mask_and_contour(img_hsv, "yellow")
        contoured_image = self.mask_and_contour(img_hsv, "orange")
        self.ContourFrame = cv2.cvtColor(contoured_image, cv2.COLOR_HSV2RGB)
        pass

    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """
        pass

    def projectGridInRGBImage(self):
        """!
        @brief      projects

                    TODO: Use the intrinsic and extrinsic matricies to project the gridpoints 
                    on the board into pixel coordinates. copy self.VideoFrame to self.GridFrame and
                    and draw on self.GridFrame the grid intersection points from self.grid_points
                    (hint: use the cv2.circle function to draw circles on the image)
                    

        """
        #print(self.grid_points)
        
        #for elements in self.grid_points:
        K = np.array([[900.543212890625, 0.0, 655.990478515625], 
                      [0.0, 900.89501953125, 353.4480285644531], 
                      [0.0, 0.0, 1.0]])
        
        # for i in range(14):

        #     for j in range(19):

        #         #print(str(i) + ", " + str(j))
        #         self.GridFrame = self.VideoFrame
                
        #         x = self.xgrid_coord[i,j]
        #         y = self.ygrid_coord[i,j]
        #         #z = self.DepthFrameRaw[y][x]
        #         #x /= z
        #         #y /= z
        #         #print(str(int(x)) + ", " + str(int(y)))
        #         cam_coord = np.array([[x,y,1]])
        #         if self.homography.size == 0:
        #             uvd = np.matmul(K,  np.transpose(cam_coord))
        #         else:
        #             uvd = np.matmul(np.linalg.inv(self.homography), np.matmul(K,  np.transpose(cam_coord)))
                # u = uvd[0,0]/1000
                # v = uvd[1,0]/1000
                #print(str(int(u)) + ", " + str(int(v)))
        self.GridFrame = self.VideoFrame.copy()
        board_points_3D = np.column_stack( (self.grid_points, np.zeros(self.grid_points.shape[0])))

        board_points_homog = np.column_stack( (board_points_3D , np.ones(self.grid_points.shape[0])))

        P = np.column_stack((K, [0.,0.,0.]))

        #pixel_locations = np.transpose(np.matmul(P, 1/self.extrinsic_matrix[2,3] * np.matmul(self.extrinsic_matrix, np.transpose(board_points_homog))))
        #pixel_locations = np.transpose(np.matmul( 1/(self.extrinsic_matrix[2,3]) * P, np.matmul(np.linalg.inv(self.extrinsic_matrix), np.transpose(board_points_homog) ) ) )

        #pixel_location_homog = np.transpose( np.matmul(self.homography, np.transpose(pixel_locations_og) ))
        #$print(pixel_locations)

        for point in board_points_homog:

            camera_cords = np.matmul(np.linalg.inv(self.extrinsic_matrix), point)

            pixel = np.matmul( 1 / camera_cords[2] * K , np.transpose([camera_cords[0],camera_cords[1],camera_cords[2]]))

            self.GridFrame = cv2.circle(self.GridFrame, (int(pixel[0]), int(pixel[1])), 5, (0,0,255), 1)
        


class ImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
         
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

        except CvBridgeError as e:
            print(e)
        self.camera.VideoFrame = cv_image


class TagImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        self.camera.TagImageFrame = cv_image


class TagDetectionListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, AprilTagDetectionArray,
                                        self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.tag_detections = data
        # for detection in data.detections:
        #     print(detection.id[0])
        #     print(detection.pose.pose.pose.position)


class CameraInfoListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, CameraInfo, self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.intrinsic_matrix = np.reshape(data.K, (3, 3))
        #print(self.camera.intrinsic_matrix)


class DepthListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
            #cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)
        self.camera.DepthFrameRaw = cv_depth
        #self.camera.DepthFrameRaw = self.camera.DepthFrameRaw/2
        self.camera.ColorizeDepthFrame()


class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_image_topic = "/tag_detections_image"
        tag_detection_topic = "/tag_detections"
        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        tag_image_listener = TagImageListener(tag_image_topic, self.camera)
        camera_info_listener = CameraInfoListener(camera_info_topic,
                                                  self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic,
                                                      self.camera)

    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Grid window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Contour window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        while True:
            rgb_frame = self.camera.convertQtVideoFrame()
            depth_frame = self.camera.convertQtDepthFrame()
            tag_frame = self.camera.convertQtTagImageFrame()
            self.camera.projectGridInRGBImage()
            self.camera.blockDetector()
            grid_frame = self.camera.convertQtGridFrame()
            contour_frame = self.camera.convertQtContourFrame()

            if ((rgb_frame != None) & (depth_frame != None)):
                self.updateFrame.emit(rgb_frame, depth_frame, tag_frame, grid_frame, contour_frame)
            time.sleep(0.03)
            if __name__ == '__main__':
                cv2.imshow(
                    "Image window",
                    cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR))
                cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                cv2.imshow(
                    "Tag window",
                    cv2.cvtColor(self.camera.TagImageFrame, cv2.COLOR_RGB2BGR))
                cv2.imshow("Grid window",
                    cv2.cvtColor(self.camera.GridFrame, cv2.COLOR_RGB2BGR))

                cv2.waitKey(3)
                time.sleep(0.03)


if __name__ == '__main__':
    camera = Camera()
    videoThread = VideoThread(camera)
    videoThread.start()
    rospy.init_node('realsense_viewer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
