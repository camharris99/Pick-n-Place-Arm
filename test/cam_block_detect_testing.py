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
            print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (
                hMin, sMin, vMin, hMax, sMax, vMax))
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


def main():

    img = cv2.imread("test\my_all_blocks.png")

    # sliders(imgitg)

    # sharpen = np.array([[0,-1,0], [-1,5,-1], [0,-1,0]])
    # img = cv2.filter2D(src=img, ddepth=-1, kernel=sharpen)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask_red = cv2.inRange(img_hsv, (174,50,50), (190, 255, 255))
    mask_orange = cv2.inRange(img_hsv, (4,167,152), (14, 250, 255))
    mask_yellow = cv2.inRange(img_hsv, (15,200,100), (35, 255, 255))
    mask_green = cv2.inRange(img_hsv, (60,50,70), (90, 255, 255))
    mask_blue = cv2.inRange(img_hsv, (100,115,90), (107, 255, 255))
    mask_purple = cv2.inRange(img_hsv, (109,43,56), (135, 255, 255))
    # mask_pink =

    mask = mask_red + mask_yellow + mask_green + mask_orange + mask_blue + mask_purple
    output = img.copy()
    output[np.where(mask==0)] = 0
    # output = cv2.medianBlur(output,3)
    output = cv2.morphologyEx(output, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
    output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8))

    cv2.imshow("image", output)
    cv2.waitKey(0)

if __name__ == "__main__":
    main()
