import cv2
import numpy as np
import freenect

lowerRange = np.array([90, 71, 66])  # B G R
upperRange = np.array([255, 255, 255])


def nothing(x):
    pass


def getContours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_NONE)
    x, y, w, h = 0, 0, 0, 0
    for countour in contours:
        area = cv2.contourArea(countour)
        if area > 50:
            cv2.drawContours(imgResult, countour, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(countour, True)
            approx = cv2.approxPolyDP(countour, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
    return x + w // 2, y


def get_video():
    array, _ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


cv2.namedWindow("Color Picker")
cv2.resizeWindow("Color Picker", 600, 300)
cv2.createTrackbar("RED Min", "Color Picker", 0, 255, nothing)
cv2.createTrackbar("RED Max", "Color Picker", 255, 255, nothing)
cv2.createTrackbar("GREEN Min", "Color Picker", 0, 255, nothing)
cv2.createTrackbar("GREEN Max", "Color Picker", 255, 255, nothing)
cv2.createTrackbar("BLUE Min", "Color Picker", 0, 255, nothing)
cv2.createTrackbar("BLUE Max", "Color Picker", 255, 255, nothing)

kernel = np.ones((5, 5), np.uint8)

while 1:
    """ lowerRange[0] = cv2.getTrackbarPos("BLUE Min", "Color Picker")
    lowerRange[1] = cv2.getTrackbarPos("GREEN Min", "Color Picker")
    lowerRange[2] = cv2.getTrackbarPos("RED Min", "Color Picker")
    upperRange[0] = cv2.getTrackbarPos("BLUE Max", "Color Picker")
    upperRange[1] = cv2.getTrackbarPos("GREEN Max", "Color Picker")
    upperRange[2] = cv2.getTrackbarPos("RED Max", "Color Picker") """
    img = get_video()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowerRange, upperRange)
    mask = cv2.morphologyEx(mask.copy(), cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask.copy(), kernel, iterations=2)
    imgResult = img.copy()
    xPos, yPos = getContours(mask)
    imgResult = cv2.putText(imgResult, str(xPos), (50, 50),
                            cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
    imgResult = cv2.putText(imgResult, str(yPos), (130, 50),
                            cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
    cv2.imshow("Live", imgResult)
    cv2.imshow("Mask", mask)
    print(upperRange)
    print(lowerRange)
    cv2.waitKey(0)
