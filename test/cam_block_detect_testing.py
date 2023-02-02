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

def mask_and_contour(output,color):
    if (color == "red"):
        mask1 = cv2.inRange(output, (174,50,50), (182, 255, 255))
        mask2 = cv2.inRange(output, (0,50,50), (3, 255, 255))
        mask = mask1 + mask2
        contour_color = (0,0,255)
    elif (color == "orange"):
        mask = cv2.inRange(output, (4,140,149), (12, 246, 218))
        contour_color = (0,165,255)
    elif (color == "yellow"):
        mask = cv2.inRange(output, (20,135,160), (27, 255, 255))
        contour_color = (0,250,250)
    elif (color == "green"):
        mask = cv2.inRange(output, (60,50,70), (90, 255, 255))
        contour_color = (0,255,0)
    elif (color == "blue"):
        mask = cv2.inRange(output, (100,115,90), (107, 255, 255))
        contour_color = (255,0,0)
    elif (color == "purple"):
        mask = cv2.inRange(output, (109,43,56), (135, 255, 255))
        contour_color = (128,0,128)

    output[np.where(mask==0)] = 0
    output = cv2.medianBlur(output,3)
    output = cv2.morphologyEx(output, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
    output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8))

    _, output = cv2.threshold(output, 70,255, cv2.THRESH_BINARY)
    output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    contours, _ = cv2.findContours(output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)

    for i in range(len(contours)):
        cv2.drawContours(output, contours, i, contour_color, 3)

    moments = get_moments(output, contours)
    return output

def get_moments(output, contours):
    pass

def color_seg(img):

    img = cv2.imread("test\my_all_blocks.png")

    # sliders(img)

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    output = mask_and_contour(img_hsv.copy(), "red")
    
    cv2.imshow("image", output)
    cv2.waitKey(0)



def main():

    img_color = cv2.imread("test\my_all_blocks.png")
    img_depth = cv2.imdecode

    # sliders(img_color)

    color_seg(img_color)

if __name__ == "__main__":
    main()
