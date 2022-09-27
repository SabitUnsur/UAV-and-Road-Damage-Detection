import numpy as np
import cv2 as cv

cap = cv.VideoCapture('Test1.mp4')

while (cap.isOpened()):

    ret, frame = cap.read()
    font = cv.FONT_HERSHEY_COMPLEX
    _, threshold = cv.threshold(frame, 110, 255, cv.THRESH_BINARY)

    # Detecting contours in image.
    contours, _ = cv.findContours(threshold, cv.RETR_TREE,
                                   cv.CHAIN_APPROX_SIMPLE)

    #Going through every contours found in the image.
    for cnt in contours:

        approx = cv.approxPolyDP(cnt, 0.009 * cv.arcLength(cnt, True), True)

        # draws boundary of contours.
        cv.drawContours(frame, [approx], 0, (0, 0, 255), 5)

        #Used to flatted the array containing
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
                    # text on topmost co-ordinate.
                    cv.putText(frame, "Arrow tip", (x, y),
                                font, 0.5, (255, 0, 0))
                else:
                    # text on remaining co-ordinates.
                    cv.putText(frame, string, (x, y),
                                font, 0.5, (0, 255, 0))
            i = i + 1

    # Showing the final image.
    cv.imshow('image2', frame)

    # Exiting the window if 'q' is pressed on the keyboard.
    if cv.waitKey(0) & 0xFF == ord('q'):
        cv.destroyAllWindows()

    cv.imshow('frame', frame)
    if cv.waitKey(35) & 0xFF == ord('q'):
        break


