import numpy as np
import cv2 as cv

cap = cv.VideoCapture('Test1.mp4')

while (cap.isOpened()):

    ret, frame = cap.read()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    cv.imshow('frame', frame)
    if cv.waitKey(35) & 0xFF == ord('q'):
        break

    edged = cv.Canny(gray, 0, 10)
    cv.imshow('Edged', edged)

    contours, hierarchy = cv.findContours(edged,
                                          cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    print("Number of Contours found = " + str(len(contours)))
    cv.drawContours(frame, contours, -1, (0, 255, 0), 3)

cap.release()
cv.destroyAllWindows()


frame = np.zeros((512,512,3),dtype = np.uint8) + 255

cv.circle(frame,(100,100),60,(0,0,0),-1)
cv.circle(frame,(350,300),20,(0,0,0),-1)

hsv=cv.cvtColor(frame,cv.COLOR_BGR2HSV)

lower_black=np.array([0,0,0])
upper_black=np.array([1,1 , 1])

mask=cv.inRange(hsv,lower_black,upper_black)

blackcnts=cv.findContours(mask.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE) [-2]

if len(blackcnts)>0:
    black_area=max(blackcnts,key=cv.contourArea)
    (xg,yg,wg,hg)=cv.boundingRect(black_area)
    cv.rectangle(frame,(xg,yg),(xg+wg,yg+hg),(0,255,0),2)


cv.release()
cv.destroyAllWindows()


x=0


for x in range(100):
        y=0
        for y in range(100):
            a=x
            b=y
            renk=frame.item[a,b,0]
            print(renk)   

        x=x+1
