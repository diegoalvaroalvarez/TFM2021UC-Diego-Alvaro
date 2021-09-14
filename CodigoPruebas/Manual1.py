import pygame
from time import sleep, time
from djitellopy import tello
import cv2

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



def getkeyboardinput():
    lr, fb, ud, yv = 0, 0, 0, 0  # left-right forward-backward up-down yaw-velocity
    global camera
    speed = 20  # cm/s

    aspeed = 70  # degrees/s 45 gira  cada vez


    if getKey('LEFT'):

        lr = -speed


    elif getKey('RIGHT'):

        lr = speed

    if getKey('UP'):

        fb = speed

    elif getKey('DOWN'):

        fb = -speed


    if getKey('w'):

        ud = speed

    elif getKey('s'):

        ud = -speed

    if getKey('a'):

        yv = -aspeed

    elif getKey('d'):

        yv = aspeed

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

    elif getKey('f'):
        cv2.imwrite(f'Fotografias/fotografiadrone{time()}.jpg', frame)
        sleep(0.2)

    elif getKey('b'):
        print(me.get_battery())

    elif getKey('q'):
        me.end()

    sleep(0.25)


    return [lr, fb, ud, yv]

init()
me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()
camera = 1

while True:

    vals=getkeyboardinput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    sleep(0.25)
    frame = me.get_frame_read().frame
    img = cv2.resize(frame, (480, 360))
    cv2.imshow("DJI TELLO", img)

    cv2.waitKey(1)

# Clean up
cv2.destroyAllWindows()