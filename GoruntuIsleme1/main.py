# Python program for Detection of a
# specific color(blue here) using OpenCV with Python
import cv2
import numpy as np

# Webcamera no 0 is used to capture the frames
cap = cv2.VideoCapture("Test1.mp4")

# This drives the program into an infinite loop.
while (1):

    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gri = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    sensitivity = 52
    lower_white = np.array([0, 0, 255 - sensitivity])
    upper_white = np.array([255, sensitivity, 255])
    max = gri.max(axis=(0, 1))
    # gri=cv2.inRange(gri,max-10,255)
    print(max)
    # break

    res = np.zeros(gri.shape)
    norm = cv2.normalize(gri, res, 128, 30, cv2.NORM_MINMAX)

    gri = cv2.GaussianBlur(gri, (5, 5), cv2.BORDER_DEFAULT)
    ret, thr = cv2.threshold(norm, 110, max, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thr,
                                           cv2.RETR_LIST,
                                           cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (255, 0, 0), -1)

    # res = cv2.bitwise_and(frame,frame, blue_mask= blue_mask)
    cv2.imshow('gri', frame)
    cv2.imshow('thr', thr)
    cv2.imshow('norm', norm)
    # cv2.imshow('mask',blue_mask)
    # cv2.imshow('res',res)

    k = cv2.waitKey(35) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()
cap.release()