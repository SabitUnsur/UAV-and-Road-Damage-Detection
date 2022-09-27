# Python program for Detection of a
# specific color(blue here) using OpenCV with Python
import cv2
import numpy as np

# Webcamera no 0 is used to capture the frames
from cv2.gapi import kernel

cap = cv2.VideoCapture("Test17.mp4")

font = cv2.FONT_HERSHEY_COMPLEX
# This drives the program into an infinite loop.

def DamageDetection(RGBDeger):

    for l in range(y, y+h):

        for d in range(x, x+w):

            #print(frame[l,d][0])

            if np.all(frame[l,d] > max) :
                #print(" Hasarlı Bölge Tespit Edildi")
                return;

            if  np.all(frame[l,d] < min):
                #print(" Hasarlı Bölge Tespit Edildi")
                return;
            frame[l,d] = [0,0,0]

while (1):

    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gri = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    sensitivity = 52

    lower_white = np.array([0, 0, 255 - sensitivity])
    upper_white = np.array([255, sensitivity, 255])

    max = gri.max(axis=(0, 1))
    min = gri.min(axis=(0,1))

    # gri=cv2.inRange(gri,max-10,255)
    #print(max)
    #print(min)

    # break

    res = np.zeros(gri.shape)
    norm = cv2.normalize(gri, res, 128, 30, cv2.NORM_MINMAX)

    gri = cv2.GaussianBlur(gri, (5, 5), cv2.BORDER_DEFAULT)
    ret, thr = cv2.threshold(norm, 105, max, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(thr,
                                           cv2.RETR_LIST,
                                           cv2.CHAIN_APPROX_SIMPLE)

    kernel = np.ones((5, 5), 'uint8')
    cv2.dilate(frame, kernel, iterations=1)
    cv2.drawContours(frame, contours, -1, (255, 0, 0), -1)

    cnts=contours

    for cnt in cnts:
        if cv2.contourArea(cnt) > 0:  # filter small contours
            x, y, w, h = cv2.boundingRect(cnt)  # offsets - with this you get 'mask'
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255,0 , 0), 2)

            #cv2.imshow('cutted contour', frame[y:y + h, x:x + w])
            #print('Average color (BGR): ', np.array(cv2.mean(frame[y:y + h, x:x + w])).astype(np.uint8))

            RGBDeger=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            #print(RGBDeger)
            DamageDetection(RGBDeger)

            approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)

            # draws boundary of contours.
            #cv2.drawContours(frame, [approx], 0, (0, 0, 255), 5)

            #Used toflatted the array containing
            # the co-ordinates of the vertices.
            n = approx.ravel()
            i = 0

            for j in n:
                if (i % 2 == 0):
                    x = n[i]
                    y = n[i + 1]

                    # String containing the co-ordinates.
                    string = str(x) + " " + str(y)

                    if (i == 0):
                        #text on topmost co-ordinate.
                        cv2.putText(frame, "", (x, y),
                                    font, 0.5, (255, 0, 0))
                        print( "Hasarlı Bolge",
                                    font, 0.5)
                    else:
                        "text on remaining co-ordinates."
                        cv2.putText(frame, string, (x, y),
                                    font, 0.5, (0, 255, 0))
                        print(string,"", (x, y),
                                    font, 0.5)
                i = i + 1

            #print('Average color (RGB): ', RGBDeger)

    # res = cv2.bitwise_and(frame,frame, blue_mask= blue_mask)
    cv2.imshow('gri', frame)
    cv2.imshow('thr', thr)
    cv2.imshow('norm', norm)
    # cv2.imshow('mask',blue_mask)
    # cv2.imshow('res',res)-ü

    cv2.waitKey(1)

cv2.destroyAllWindows()
cap.release()