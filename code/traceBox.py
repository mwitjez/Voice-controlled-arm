import time
import cv2
import numpy as np
from PCA9685 import PCA9685
import snowboydecoder
import sys
import signal
from picamera.array import PiRGBArray
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)

lowerRange = np.array([121, 70, 68])  # B G R
upperRange = np.array([255, 255, 255])

xPos = 0
yPos = 0

pwm = PCA9685(0x40)
pwm.setPWMFreq(50)


def nothing(x):
    pass


def getContours(img):
    img, contours, hierarchy = cv2.findContours(
        img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    x, y, w, h = 0, 0, 0, 0
    for countour in contours:
        area = cv2.contourArea(countour)
        if area > 300:
            # cv2.drawContours(imgResult, countour, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(countour, True)
            approx = cv2.approxPolyDP(countour, 0.02*peri, True)
            x, y, w, h = cv2.boundingRect(approx)
    return x+w//2, y

# detection


interrupted = False


def signal_handler(signal, frame):
    global interrupted
    interrupted = True


def interrupt_callback():
    global interrupted
    return interrupted


def pickItUp():
    print("picking...")
    # allow the camera to warmup
    time.sleep(0.1)
    # grab an image from the camera
    camera.capture(rawCapture, format="bgr")
    img = rawCapture.array
    imgRotated = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    hsv = cv2.cvtColor(imgRotated, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowerRange, upperRange)
    xPos, yPos = getContours(mask)
    pwm.setServoPulse(14, 1500)
    pwm.setServoPulse(13, 1500)
    pwm.setServoPulse(15, 1750)
    pwm.setServoPulse(12, 2000)
    time.sleep(1)
    pwm.setServoPulse(15, -1.7*xPos+2100)
    time.sleep(1)
    for x in range(200):
        pwm.setServoPulse(13, 1600+x)  # down
        pwm.setServoPulse(14, 1500+x*3)
        time.sleep(0.02)
    pwm.setServoPulse(12, 1300)  # close
    time.sleep(1)
    pwm.setServoPulse(13, 1800)  # up
    pwm.setServoPulse(14, 1200)
    pwm.setServoPulse(15, 1750)
    rawCapture.truncate(0)


def dropIt():
    print("dropping")
    pwm.setServoPulse(12, 2000)


models = ['pickItUp.pmdl', 'dropIt.pmdl']

# capture SIGINT signal, e.g., Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

sensitivity = [0.5]*len(models)
detector = snowboydecoder.HotwordDetector(models, sensitivity=sensitivity)
callbacks = [lambda: pickItUp(),
             lambda: dropIt()]
print('Listening... Press Ctrl+C to exit')

# hotword detection
detector.start(detected_callback=callbacks,
               interrupt_check=interrupt_callback,
               sleep_time=0.03)
