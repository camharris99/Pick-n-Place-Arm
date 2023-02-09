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

class Block():
    """!
    @brief This class describes a shape in the workspace
    """

    def __init__(self):
        """!
        @brief Constructs an instance of a block object
        """

        self.XYZ = self.assignXYZ()
        self.shape = "unassigned"
        self.color = "unassigned"
        self.angle = 0.

    def assignXYZ(self):
        """!
        @brief Assigns XYZ coordinates to block object
        """
        self.XYZ = Camera.returnBlockXYZ()

    def assignShape(self):
        """!
        @brief Assigns shape to block object
        """
        self.shape = Camera.returnBlockShape()


    def assignColor(self):
        """!
        @brief Assigns color to block object
        """    

        self.color = Camera.returnBlockColor()
    
    def assignColor(self):
        """!
        @brief Assigns color to block object
        """   

class Camera():
    """!
    @brief      This class describes a camera.
    """
    def __init__(self):
        """!
        @brief      Constructs a new instance.
        """
        self.block_coords = np.zeros([4,1])
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
        self.num_blocks = 0

    def returnBlockXYZ(self):
        """!
        @brief reeturns array of block coordinates - to be used for block class
        """

        return self.block_coords

    def returnBlockShape(self):
        """!
        @brief reeturns array of block coordinates - to be used for block class
        """

        return self.block_shapes

    def returnBlockColor(self):
        """!
        @brief reeturns array of block coordinates - to be used for block class
        """

        return self.block_colors

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
        """
        @brief      Detect and draw contours on an image
        
        @param      image must be in HSV
        """

        copy = image.copy() #create a copy of the image

        # specify the HSV ranges to keep and recognize as the color provided
        # note that red in cv2 is 0-179, so need to create two masks and sum
        # cv2.inRange will create a binary array where zeros correspond to
        # # somewhere that doesn't fall within the specified HSV range

        if (color == "red"):
            mask1 = cv2.inRange(copy, (170,140,80), (179, 255, 255))
            mask2 = cv2.inRange(copy, (0,55,70), (3, 255, 255))
            mask = mask1 + mask2
            contour_color = (175,255,255)
        elif (color == "orange"):
            mask = cv2.inRange(copy, (3,124,154), (15, 255, 255))
            contour_color = (8,250,250)
        elif (color == "yellow"):
            mask = cv2.inRange(copy, (20,135,160), (27, 255, 255))
            contour_color = (23,200,250)
        elif (color == "green"):
            mask = cv2.inRange(copy, (60,50,80), (90, 255, 255))
            contour_color = (75,255,255)
        elif (color == "blue"):
            mask = cv2.inRange(copy, (93,93,93), (110, 255, 255))
            contour_color = (102,255,255)
        elif (color == "purple"):
            mask = cv2.inRange(copy, (109,0,0), (150, 255, 255))
            contour_color = (115,120,255)

        # this is where the magic happens. Set everywhere in the image that
        # # corresponds to a zero in the mask also to zero
        copy[np.where(mask==0)] = [0,0,0]

        # median blur filter helps reduce noise. the number 3 corresponds to kernel size
        # increasing the second argument will cause more blurring
        copy = cv2.medianBlur(copy,5)

        # we learned these in lecture. they reduce false positives on the play field and 
        # # also will fill in the image when its grainy
        copy = cv2.morphologyEx(copy, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
        copy = cv2.morphologyEx(copy, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))

        # setting a threshold to make the contours easier to find
        _, copy = cv2.threshold(copy, 70,255, cv2.THRESH_BINARY)

        # this is all a bit annoying, need to convert from HSV to RGB so we can convert to gray
        gray_copy = cv2.cvtColor(copy, cv2.COLOR_HSV2RGB)
        gray_copy = cv2.cvtColor(gray_copy, cv2.COLOR_RGB2GRAY)

        # i misspoke, this is actually where the magic happens. we can draw all of the contours
        # # based on the processed image, which should have only one color present at this point
        _, contours, _ = cv2.findContours(gray_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        moments = []
        
        # needs to be a reverse loop so we can delete entries that dont meet a criteria
        # basically, loop through each contour and process the centroid/area etc of that contour
        for i in range(len(contours)-1, -1, -1):

            # moment stores all the information we can use to get centroid and stuff
            moment = cv2.moments(contours[i])

            # u,v are calculated from the moments. these are the centroid of the contour in mouse coordinates
            u = int(moment['m10']/moment['m00'])
            v = int(moment['m01']/moment['m00'])

            # use depth data to get the mouse coordinate depth of the contour's centroid
            d = self.DepthFrameRaw[v][u]

            # convert mouse coordinates of centroid to camera coordinates, then to world coordinates
            cam_coords = d*np.matmul(np.linalg.inv(self.intrinsic_matrix), [u, v, 1])
            world_coords = np.matmul(self.extrinsic_matrix, np.append(np.array(cam_coords),1))

            #checks bounds out of board
            if (world_coords[0] > 450 or world_coords[0] < -450 or world_coords[1] < -150 or world_coords[1] > 450):
                del contours[i]
                continue

            # calculating area of the contour. this can be compared to perimeter or something to determine shape
            contour_area = cv2.contourArea(contours[i])

            # check that area is at least as large as the smallest block
            if (contour_area < 400):
                del contours[i]
                continue

            # add the current moment to the front of the list of moments
            moments.insert(0, moment)

            # draw a circle on the image at the centroid
            cv2.circle(image, (u,v), radius = 5, color=(0,0,0), thickness=-1) 

            # find the rectangle with the minimum area of the contour, along with the bounding points
            rect = cv2.minAreaRect(contours[i])
            box = cv2.boxPoints(rect)
            box = np.intp(box)

            # draw the contour using the rectangle of minimum area
            cv2.drawContours(image,[box],0,contour_color,2)

            # store orientation to flat of the contour
            orientation = rect[2]

            # store the last element of the world coordinates as the orientation of the block
            world_coords[-1] = orientation
            world_coords = np.expand_dims(world_coords, axis=1)
            
            # add the world coordinates of the current contour to self.block_coords
            self.block_coords = np.hstack((self.block_coords, world_coords))

            # write the contour area on the image
            cv2.putText(image, str(cv2.contourArea(contours[i])), (box[2,0],box[2,1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), 2)

        # print(self.block_coords)

        return image, moments, contours

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        img_hsv = cv2.cvtColor(self.VideoFrame.copy(), cv2.COLOR_RGB2HSV)
        # self.sliders(img_hsv)
        self.block_coords = np.zeros([4,1])
        contoured_image, red_moments, red_contours = self.mask_and_contour(img_hsv, "red")
        contoured_image, green_moments, green_contours = self.mask_and_contour(img_hsv, "green")
        contoured_image, blue_moments, blue_contours = self.mask_and_contour(img_hsv, "blue")
        contoured_image, purple_moments, purple_contours = self.mask_and_contour(img_hsv, "purple")
        contoured_image, yellow_moments, yellow_contours = self.mask_and_contour(img_hsv, "yellow")
        contoured_image, orange_moments, orange_contours = self.mask_and_contour(img_hsv, "orange")
        # grid_box = np.array([[-450,-150], [-450, 450], [450, 450], [450, -150]])
        self.num_blocks = len(red_contours) + len(green_contours) + len(blue_contours) + len(purple_contours) \
            + len(yellow_contours) + len(orange_contours)
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
