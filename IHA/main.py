from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
import RPi.GPIO as GPIO
from pymavlink import mavutil
import cv2
import numpy as np
from math import sqrt

# gerçek baglantı >>> baud=115200, port= /dev/ttyACM0
iha = connect('127.0.0.1:14550', wait_ready=True)
print("baglanti sağlandı")

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
servo = GPIO.PWM(12, 50)


def velocity(velocity_x, velocity_y, velocity_z, iha):
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0)
    iha.send_mavlink(msg)


def takeoff(irtifa):
    while iha.is_armable is not True:
        print("İHA arm edilebilir durumda değil.")
        time.sleep(1)

    print("İHA arm edilebilir.")

    iha.mode = VehicleMode("GUIDED")

    iha.armed = True

    while iha.armed is not True:
        print("İHA arm ediliyor...")
        time.sleep(0.5)

    print("İHA arm edildi.")

    iha.simple_takeoff(irtifa)

    while iha.location.global_relative_frame.alt < irtifa * 0.9:
        print("İHA hedefe yükseliyor.")
        time.sleep(1)


komut = iha.commands

komut.clear()
time.sleep(1)

takeoff(10)

# TAKEOFF
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0,
            0, 0, 0, 10))
print("Takeoff oldu")

# 1. tur
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7899959, 30.4822889, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7899092, 30.4825290, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7898057, 30.4826885, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7895935, 30.4828589, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7895287, 30.4830573, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896123, 30.4833135, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7898172, 30.4833537, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7899897, 30.4833041, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.790205, 30.4832076, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7903608, 30.4830828, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7903691, 30.4828495, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7902897, 30.4825920, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7901454, 30.4824552, 10))

# havuz orta nokta

# dalış
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896690, 30.4827630, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896690, 30.4827630, 5))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896690, 30.4827630, 3))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896690, 30.4827630, 2))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896690, 30.4827630, 1.7))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896690, 30.4827630, 1.8))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896690, 30.4827630, 1.5))

# kalkış
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896690, 30.4827630, 4))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896690, 30.4827630, 10))

# beyaz alana giden yol
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7895694, 30.4829286, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7895506, 30.4831405, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7896113, 30.4832545, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7897001, 30.4833417, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7898695, 30.4833229, 10))

# beyaz alan (görüntü işleme yapılacak)
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38, 7899740, 30, 4832719, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38, 7899740, 30, 4832719, 5))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7900587, 30.4832424, 10))

komut.upload()
print("Komutlar yükleniyor...")
komut.next = 0
iha.mode = VehicleMode("AUTO")

merkezx = 320
merkezy = 240

cap = cv2.VideoCapture(0)
a = 0

while a == 0:
    next_waypoint = komut.next

    if next_waypoint is 30:
        komut.clear()
        komut.upload()
        komut.next = 0
        iha.mode = VehicleMode("GUIDED")

        while True:
            ret, frame = cap.read()
            frame = cv2.flip(frame, 1)
            frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            frame_lab = cv2.medianBlur(frame_lab, 3)
            frame_lab = cv2.cvtColor(frame_lab, cv2.COLOR_BGR2Lab)
            lower_red = np.array([20, 150, 150])
            upper_red = np.array([190, 255, 255])
            masked_frame = cv2.inRange(frame_lab, lower_red, upper_red)
            masked_frame = cv2.GaussianBlur(masked_frame, (5, 5), 2, 2)
            circles = cv2.HoughCircles(masked_frame, cv2.HOUGH_GRADIENT, 1, masked_frame.shape[0] / 8, param1=100,
                                       param2=23, minRadius=15, maxRadius=60)
            cv2.circle(frame, (320, 240), 3, (255, 255, 255), -1)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:

                    center = (i[0], i[1])
                    cv2.circle(frame, center, 1, (0, 255, 0), 3)
                    radius = i[2]
                    cv2.circle(frame, center, radius, (0, 255, 0), 3)
                    cv2.putText(frame, "Merkez", (center), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

                    redx, redy = center
                    dx = abs(merkezx - redx)
                    dy = abs(merkezy - redy)
                    R = sqrt(dx * dx + dy * dy)
                    print(R)

                    # CENTER[0] =320 CENTER[1]=240
                    if (center[0] < 290):
                        velocity(0, -0.5, 0, iha)

                    elif (center[0] > 350):
                        velocity(0, 0.5, 0, iha)

                    if (center[1] < 210):
                        velocity(0.5, 0, 0, iha)

                    elif (center[1] > 270):
                        velocity(-0.5, 0, 0, iha)


                    else:
                        print("Dur")
                        if R <= 50:
                            a = a + 1
                            print("servo çalıştır")
                            servo.start(12)
                            time.sleep(10)
                            break
                        print("forum bitti")
            cv2.imshow('Kamera', frame)
            if (cv2.waitKey(1) & 0xFF == ord('q') or a == 1):
                break

cap.release()
cv2.destroyAllWindows()

komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.790181, 30.482993, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7902709, 30.4829058, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7903398, 30.4828522, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7903398, 30.4827744, 10))

komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7903137, 30.4827248, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7902677, 30.4826818, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.7902249, 30.4825893, 10))
komut.add(
    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0,
            0, 38.789997, 30.4822896, 10))

komut.upload()
print("Komutlar yükleniyor...")
komut.next = 0
iha.mode = VehicleMode("AUTO")