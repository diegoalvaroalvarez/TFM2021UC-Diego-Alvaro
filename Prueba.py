import cv2
import numpy as np
from djitellopy import tello
from time import sleep

me = tello.Tello()
me.connect()
print(me.get_battery())

me.streamon()



def drawpoints(img, points, angulo=0.0):

    for point in points:

        cv2.circle(img, point, 5, (240, 240, 240), cv2.FILLED)  # bgr color

    cv2.drawMarker(img, point, (255, 0, 0), cv2.MARKER_STAR, 25, 4)

    cv2.putText(img, f'({round((points[-1][0] - 1500)/30, 2)},{round((-points[-1][1] + 1500)/30, 2)},) m {angulo}gr',
                (points[-1][0] + 10, points[-1][1] + 30), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 2, (255, 0, 255), 6)
i=0
me.takeoff()
sleep(5)
while True:
    # drone = me.get_frame_read().frame
    # drone = cv2.resize(drone, (640,360))
    # cv2.imshow("DJI TELLO", drone)
    mapeado = np.zeros((3000, 3000, 3), np.uint8)
    points=[(1500, 1500), (1500, 1530), (1500, 1560), (1530, 1560), (1530, 1590)]
    drawpoints(mapeado, points)
    mapeado = cv2.resize(mapeado, (500, 500))
    cv2.imshow('Mapeado', mapeado)
    me.go_xyz_speed(30, 0, 0, 15)
    sleep(2)
    me.go_xyz_speed(20, 20, 0, 15)
    sleep(2)
    me.rotate_clockwise(45)
    sleep(2)
    me.rotate_clockwise(15)
    sleep(2)
    me.end()
    sleep(5)
    break
    cv2.waitKey(1)