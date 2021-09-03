import pygame
from time import sleep, time
from djitellopy import tello
import cv2
import numpy as np
import math

def init():

    pygame.init()

    win = pygame.display.set_mode((200, 200))

def getKey(keyName):
    ans = False

    for eve in pygame.event.get(): pass

    keyInput = pygame.key.get_pressed()

    myKey = getattr(pygame, 'K_{}'.format(keyName))

    # print('K_{}'.format(keyName))

    if keyInput[myKey]:

        ans = True

    pygame.display.update()

    return ans

######## PARAMETROS EMPIRICOS ###########

fSpeed = 40 / 10  # Forward Speed in cm/s   (20 cm/s) 115 / 10

aSpeed = 1800 / 10  # Angular Speed Degrees/s  (45 d/s) 60 gira 30 cada vez

interval = 0.25

dInterval = fSpeed * interval

aInterval = aSpeed * interval

###############################################
x, y = 250, 250
a = 0
yaw = 0


def getkeyboardinput():
    lr, fb, ud, yv = 0, 0, 0, 0  # left-right forward-backward up-down yaw-velocity
    d = 0
    global camera, x, y, yaw, a

    speed = 20  # cm/s

    aspeed = 70  # degrees/s 45 gira  cada vez

    if getKey('LEFT'):

        lr = -speed

        d = dInterval

        a = -180

    elif getKey('RIGHT'):

        lr = speed

        d = -dInterval

        a = 180

    if getKey('UP'):

        fb = speed

        d = dInterval

        a = -90

    elif getKey('DOWN'):

        fb = -speed

        d = -dInterval

        a = 270

    if getKey('w'):

        ud = speed

    elif getKey('s'):

        ud = -speed

    if getKey('a'):

        yv = -aspeed

        yaw -= aInterval

    elif getKey('d'):

        yv = aspeed

        yaw += aInterval

    if getKey('l'):
        me.land()
        sleep(2)

    elif getKey('t'):
        me.takeoff()
        sleep(0.25)

    elif getKey('c'):
        if camera == 1:
            me.streamoff()
            camera = 0
        elif camera == 0:
            me.streamon()
            camera = 1

    elif getKey('p'):
        cv2.imwrite(f'Fotografias/fotografiadrone{time()}.jpg', frame)
        sleep(0.2)

    elif getKey('b'):
        print(me.get_battery())

    elif getKey('q'):
        cv2.imwrite(f'Mapeado/mapeadofinal{time()}.jpg', mapeado)
        sleep(0.2)
        np.save('datos/posicion.npy', pos)
        np.save('datos/angulo.npy', yaw)
        np.save('datos/recorrido.npy', points)
        print('¡Hasta la proxima')
        me.end()

    sleep(interval)

    if yaw > 180:
        yaw = yaw - 360 * (yaw // 180)
    elif yaw < -180:
        yaw = yaw + 360 * (-yaw // 180)

    a += yaw

    x += int(d * math.cos(math.radians(a)))

    y += int(d * math.sin(math.radians(a)))

    return [lr, fb, ud, yv]

def drawpoints(img, points, pos, yaw=0.0):

    for point in points:

        cv2.circle(img, point, 1, (240, 240, 240), cv2.FILLED)  # bgr color circulos blancos


    cv2.putText(img, f'({round((points[-1][0] - 250)/10, 2)},{round((-points[-1][1] + 250)/10, 2)},{me.get_height()/100}) m {yaw}gr',
                (points[-1][0] + 3, points[-1][1] + 5), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 0, 255), 1)

    cv2.drawMarker(img, pos, (255, 0, 0), cv2.MARKER_STAR, 6, 1) # Estrella azul

# Comienzo del programa
nuevo = str(input("El mapeado de obstaculos en nuevo? (SI/NO): "))
if nuevo == "SI":
    pos = (250, 250)
    yaw = 0
    points = [(0, 0)]

else:
    obstaculos = np.load('datos/obstaculos.npy')
    pos = np.load('datos/posicion.npy')
    yaw = np.load('datos/angulo.npy')
    points = np.load('datos/recorrido.npy')

# Inicializacion
init()
me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()
camera=1

while True:
    mapeado = np.zeros((500, 500, 3), np.uint8)

    [vals, pos, yaw]=getkeyboardinput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    sleep(0.25)

    if points[-1][0] != pos[0] or points[-1][1] != pos[1]:
        points.append(pos)

    # Mapeado
    drawpoints(mapeado, points, pos, yaw)
    cv2.imshow('Mapeado', mapeado)

    # Camara drone
    frame = me.get_frame_read().frame
    img = cv2.resize(frame, (480, 360))
    cv2.imshow("DJI TELLO", img)

    cv2.waitKey(1)

# Clean up
cv2.destroyAllWindows()