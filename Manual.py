
import KeyPressModule as kp

from time import sleep


import math

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

kp.init()


def getkeyboardinput():
    lr, fb, ud, yv = 0, 0, 0, 0  # left-right forward-backward up-down yaw-velocity

    speed = 20  # cm/s

    aspeed = 70  # degrees/s 45 gira  cada vez

    global x, y, yaw, a

    d = 0
    if kp.getKey('LEFT'):

        lr = -speed

        d = dInterval

        a = -180

    elif kp.getKey('RIGHT'):

        lr = speed

        d = -dInterval

        a = 180

    if kp.getKey('UP'):

        fb = speed

        d = dInterval

        a = -90

    elif kp.getKey('DOWN'):

        fb = -speed

        d = -dInterval

        a = 270

    if kp.getKey('w'):

        ud = speed

    elif kp.getKey('s'):

        ud = -speed

    if kp.getKey('a'):

        yv = -aspeed

        yaw -= aInterval

    elif kp.getKey('d'):

        yv = aspeed

        yaw += aInterval

    sleep(interval)

    if yaw > 180:
        yaw = yaw - 360 * (yaw // 180)
    elif yaw < -180:
        yaw = yaw + 360 * (-yaw // 180)

    a += yaw

    x += int(d * math.cos(math.radians(a)))

    y += int(d * math.sin(math.radians(a)))

    return [lr, fb, ud, yv], (x, y), yaw