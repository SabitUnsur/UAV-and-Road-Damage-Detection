import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while (True):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.bitwise_not(gray)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    roi = 0

    thresh = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY)[1]

    thresh = cv2.dilate(thresh, None, iterations=2)
    (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_SIMPLE)
    edges = cv2.Canny(frame, 170, 200)
    lines = cv2.HoughLinesP(edges, rho=1, theta=1 * np.pi / 180, threshold=191, minLineLength=250, maxLineGap=150)

    for c in cnts:
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        roi = frame[y:y + h, x:x + w]


    cv2.imshow("Security Feed", frame)
    cv2.imshow("Thresh", thresh)
    cv2.imshow("Roi", roi)

    cv2.waitKey(1)