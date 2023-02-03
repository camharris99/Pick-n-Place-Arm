# block detection testing
import numpy as np
import cv2
import sys


def nothing(x):
    pass

def sliders(img):
     # Create a window
    cv2.namedWindow('image')

    # create trackbars for color change
    # Hue is from 0-179 for Opencv
    cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
    cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
    cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
    cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
    cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
    cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

    # Set default value for MAX HSV trackbars.
    cv2.setTrackbarPos('HMax', 'image', 179)
    cv2.setTrackbarPos('SMax', 'image', 255)
    cv2.setTrackbarPos('VMax', 'image', 255)

    # Initialize to check if HSV min/max value changes
    hMin = sMin = vMin = hMax = sMax = vMax = 0
    phMin = psMin = pvMin = phMax = psMax = pvMax = 0

    output = img
    waitTime = 33

    while (1):

        # get current positions of all trackbars
        hMin = cv2.getTrackbarPos('HMin', 'image')
        sMin = cv2.getTrackbarPos('SMin', 'image')
        vMin = cv2.getTrackbarPos('VMin', 'image')

        hMax = cv2.getTrackbarPos('HMax', 'image')
        sMax = cv2.getTrackbarPos('SMax', 'image')
        vMax = cv2.getTrackbarPos('VMax', 'image')

        # Set minimum and max HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(img, img, mask=mask)

        # Print if there is a change in HSV value
        if ((phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax)):
            # print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (
                # hMin, sMin, vMin, hMax, sMax, vMax))
            phMin = hMin
            psMin = sMin
            pvMin = vMin
            phMax = hMax
            psMax = sMax
            pvMax = vMax

        # Display output image
        cv2.imshow('image', output)

        # Wait longer to prevent freeze for videos.
        if cv2.waitKey(waitTime) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

def mask_and_contour(image, color):
    copy = image.copy()
    if (color == "red"):
        mask1 = cv2.inRange(copy, (174,100,100), (182, 255, 255))
        mask2 = cv2.inRange(copy, (0,100,100), (3, 255, 255))
        mask = mask1 + mask2
        contour_color = (175,255,255)
    elif (color == "orange"):
        mask = cv2.inRange(copy, (3,124,154), (15, 255, 255))
        contour_color = (8,250,250)
    elif (color == "yellow"):
        mask = cv2.inRange(copy, (20,135,160), (27, 255, 255))
        contour_color = (23,200,250)
    elif (color == "green"):
        mask = cv2.inRange(copy, (60,50,70), (90, 255, 255))
        contour_color = (75,255,255)
    elif (color == "blue"):
        mask = cv2.inRange(copy, (93,207,109), (108, 255, 255))
        contour_color = (102,255,255)
    elif (color == "purple"):
        mask = cv2.inRange(copy, (109,43,56), (135, 255, 255))
        contour_color = (115,120,255)

    copy[np.where(mask==0)] = [0,0,0]
    copy = cv2.medianBlur(copy,3)
    copy = cv2.morphologyEx(copy, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
    copy = cv2.morphologyEx(copy, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8))

    _, copy = cv2.threshold(copy, 70,255, cv2.THRESH_BINARY)
    gray_copy = cv2.cvtColor(copy, cv2.COLOR_HSV2BGR)
    gray_copy = cv2.cvtColor(gray_copy, cv2.COLOR_BGR2GRAY)
    contours, _ = cv2.findContours(gray_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    moments = []
    for i in range(len(contours)):
        # cv2.drawContours(image, contours, i, contour_color, 3)
        moments.append(cv2.moments(contours[i]))
        cv2.circle(image, (int(moments[i]['m10']/moments[i]['m00']),int(moments[i]['m01']/moments[i]['m00'])), radius = 5, color=(0,0,0), thickness=-1) 
        rect = cv2.minAreaRect(contours[i])
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        cv2.drawContours(image,[box],0,contour_color,2)
    return image, moments, contours

def blockDetector(img):
    """!
    @brief      Detect blocks from rgb

                TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                locations in self.block_detections
    """
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    contoured_image, red_moments, red_contours = mask_and_contour(img_hsv, "red")
    contoured_image, green_moments, green_contours = mask_and_contour(img_hsv, "green")
    contoured_image, blue_moments, blue_contours = mask_and_contour(img_hsv, "blue")
    contoured_image, purple_moments, purple_contours = mask_and_contour(img_hsv, "purple")
    contoured_image, yellow_moments, yellow_contours = mask_and_contour(img_hsv, "yellow")
    contoured_image, orange_moments, orange_contours = mask_and_contour(img_hsv, "orange")
    contoured_image = cv2.cvtColor(contoured_image, cv2.COLOR_HSV2BGR)
    cv2.imshow('image', contoured_image)
    cv2.waitKey(0)
    # self.ContourFrame = cv2.cvtColor(contoured_image, cv2.COLOR_HSV2RGB)


def main():

    img_color = cv2.imread('test\image_blue_testing.png')
    # img_depth = cv2.imdecode

    # sliders(img_color)

    blockDetector(img_color)


if __name__ == "__main__":
    main()
