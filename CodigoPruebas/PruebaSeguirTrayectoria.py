from djitellopy import tello
import Manual0 as man
import math
import os
import numpy as np
import sys
import time
import tensorflow as tf
import cv2
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM

# Valores iniales del drone
x, y = 250, 250  # posicion inicial
yaw = 0  # angulo inicial
OBSTACLE = 255
UNOCCUPIED = 0
x_dim = 500
y_dim = 500
start = (x, y)
destpx = (500, 500)
view_range = 5
map = OccupancyGridMap(x_dim, y_dim, '8N')
vals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
points = [(0, 0), (0, 0)]
obstaculos2 = []
i = 0
pos = (x, y)
lastpos = (x-1, y-1)
slam = SLAM(map=map, view_range=view_range)
new_observation = {"pos": None, "type": None}
modo = False
manual = False
automatico = True
delante = []

def convdist(posicion, modo=0):
    """
    modo 0 es m a px """
    nposicion = []
    if modo == 0:  # Convertir de m a px del mapeado
        nposicion.append(int(posicion[0]*100/10+250))
        nposicion.append(int(round(250-posicion[1] * 100 / 10)))

    elif modo == 1: #convertir px del mapeado a m
        nposicion.append(round((posicion[0]-250)*10/100, 2))
        nposicion.append(round(-(posicion[1]-250)*10/100, 2))
    npos=(nposicion[0], nposicion[1])
    return npos

def angulodegiro(pos0, pos1, posref):
    """
    Obtener el angulo de giro entre dos puntos respecto a un punto de referencia
    Devuelve el angulo entre 0-180 y el sentido horario-antihorario
    """
    ax = posref[0] - pos0[0]  # a vector pos0aposderef
    ay = posref[1] - pos0[1]
    bx = pos1[0] - posref[0] # b vector de posrefapos1
    by = pos1[1] - posref[1]
    if ax == 0:
        px = -ay
        py = 0
    elif ay == 0:
        px = 0
        py = -ax
    elif ax != -ay:
        py = -ax / (ax+ay)  # p vector perpendicular a vector a
        px = py + 1
    else:
        py = ax/ay
        px = 2

    # Get dot product of pos0 and pos1.
    _dot = (ax * bx) + (ay * by)
    _dot2 = (px * bx) + (py * by)
    # Get magnitude of pos0 and pos1.
    _magP = math.sqrt(px**2 + py**2)
    _magB = math.sqrt(bx**2 + by**2)
    _magA = math.sqrt(ax ** 2 + ay ** 2)
    _rad = math.acos(_dot / (_magA * _magB))
    _rad2 = math.acos(_dot2 / (_magP * _magB))
    # Angulo en grados.
    angle = (_rad * 180) / math.pi  # angulo entre pos 0 y pos 1
    angle2 = (_rad2 * 180) / math.pi  # angulo entre perpendicular a pos0posref y pos1
    round(angle)
    if pos0 == pos1:
        angle = 180
        sentido = 'horario'
    elif 0 < angle2 <= 90 or angle2==180 or (angle2==0 and (bx<0 or by<0)):
        sentido = 'horario'
        angle = angle
    else:
        sentido = 'antihorario'
        angle = angle

    return int(angle), sentido

def signo(num):
    if num < 0:
        return 0
    else:
        return 1

def drawobstacles(img, obstaculos):
    # Cambiar colores y formato de cada cosa
    for l in range(2, len(obstaculos)):
        for j in range(-1, 1):
            for p in range(-1, 1):
                cv2.circle(img, (obstaculos[l][0]+j, obstaculos[l][1]+p), 1, (152, 255, 255), cv2.FILLED)  # amarillo claro
    for w in range(2, len(obstaculos)):
        if obstaculos[w][2] == 4:  # Arbol
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]), (57, 143, 0), cv2.MARKER_TRIANGLE_UP, 4, 1) # verde oscuro
        elif obstaculos[w][2] == 1:  # Poste
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]), (140, 0, 0), cv2.MARKER_SQUARE, 4, 1)  # azul clarito
        elif obstaculos[w][2] == 2:  # Valla
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]), (0, 233, 255), cv2.MARKER_SQUARE, 4, 1)  # amarillo
        elif obstaculos[w][2] == 3:  # Cartel
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]), (39, 14, 34), cv2.MARKER_CROSS, 4, 1)  # morado
        elif obstaculos[w][2] == 5:  # Vehiculo
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]), (0, 0, 240), cv2.MARKER_SQUARE, 4, 1)  # rojo
        elif obstaculos[w][2] == 7:  # Persona
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]), (203, 142, 255), cv2.MARKER_TRIANGLE_DOWN, 4, 1)  # rosa
        elif obstaculos[w][2] == 8:  # Muro
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]),  (0, 128, 255), cv2.MARKER_DIAMOND, 4, 2)   # naranja
        elif obstaculos[w][2] == 6:  # Semaforo
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]), (155, 155, 155), cv2.MARKER_TILTED_CROSS, 4, 1)  # gris

def drawpoints(img, points, pos, angulo=0.0, modo = 0):
    global path
    for point in points:

        cv2.circle(img, point, 1, (240, 240, 240), cv2.FILLED)  # bgr color circulos blancos


    cv2.putText(img, f'({round((points[-1][0] - 250)/10, 2)},{round((-points[-1][1] + 250)/10, 2)},) m {angulo}gr',
                (points[-1][0] + 3, points[-1][1] + 5), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 0, 255), 1)
    if modo == 1:
        for point in path:
            cv2.circle(img, point, 1, (130, 130, 240), cv2.FILLED) #circulos rojo claro
    cv2.drawMarker(img, pos, (255, 0, 0), cv2.MARKER_STAR, 6, 1) # Estrella azul

path= [(250,250),(250,249),(250,248),(250,247),(250,246),(250,245),(251,244),(252,243),(251,242),(250,241),(251,241),(252,241),(252,242),
              (252,243),(251,244),(251,245),(251,246),(251,247),(250,247),(250,248),(250,249),(250,250)]



man.init()

me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()
j=1
while True:
    mapeado = np.zeros((500, 500, 3), np.uint8)
    if man.getKey('l'):
        me.land()
        time.sleep(2)
    if man.getKey('t'):
        me.takeoff()
        time.sleep(2)
    if man.getKey('m'):
        if modo == 0:
            i = 0
        modo = not modo
        print('Modo cambiado')
        time.sleep(0.25)
    if man.getKey('r'):
        points = [(0, 0), (0, 0)]
    if man.getKey('g'):  # nuevo destino
        # points = [(0, 0), (0, 0)]
        if modo == 1:
            i = 0

    if man.getKey('b'):
        print(me.get_battery())
        time.sleep(2)


    frame = me.get_frame_read().frame

    if modo == manual:
        if pos != lastpos:
            lastpos = pos
        [vals, pos, yaw] = man.getkeyboardinput()
        me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        time.sleep(0.25)

    elif modo == automatico:
            if len(path[i:-1]) == 1 or 0:
                print("Ha llegado a su destino, aterrice o pulse la tecla G para seleccionar un nuevo destino")
                me.send_rc_control(0, 0, 0, 0)
                time.sleep(0.25)

            else:
                i+=1
                print(i)
                pos = path[i]
                nextpos = path[-1]
                if len(path[i:-1]) > 2:
                    nextpos = path[i+1]
                lastpos = path[i - 1]
                anggiro , angsent = angulodegiro(lastpos, nextpos, pos)

                print(path[i])
                print(anggiro)

                if anggiro != 0: # and j==1:
                    # i-=1
                    #j = 0
                    if angsent == 'horario':
                        #me.rotate_clockwise(anggiro)
                        #time.sleep(3)
                        me.send_rc_control(0, 0, 0,int(anggiro/2))
                        time.sleep(2.7)
                        me.send_rc_control(0, 0, 0, 0)
                        time.sleep(0.25)
                        yaw = yaw + anggiro
                    else:
                        #me.rotate_counter_clockwise(anggiro)
                        #time.sleep(1.5)
                        me.send_rc_control(0, 0, 0, int(-anggiro/2))
                        time.sleep(2.3)
                        me.send_rc_control(0, 0, 0, 0)
                        time.sleep(0.25)
                        yaw = yaw - anggiro

                    if yaw > 180:
                        yaw = yaw - 360 * (yaw // 180)
                    elif yaw < -180:
                        yaw = yaw + 360 * (-yaw // 180)
                #else:
                    #j=1
                me.send_rc_control(0, 30, 0, 0)
                print('avanza')
                time.sleep(0.33)

                #lastpos=path[i]
                me.send_rc_control(0, 0, 0, 0)
                time.sleep(0.25)
    if points[-1][0] != pos[0] or points[-1][1] != pos[1]:
        points.append(pos)

    drawpoints(mapeado, points, pos, yaw, modo)
    cv2.imshow('Mapping', mapeado)
    # Camara drone
    img = cv2.resize(frame, (480, 360))
    cv2.imshow("DJI TELLO", img)
    time.sleep(0.1)
    if man.getKey('q'):

        print('¡Hasta la proxima')
        me.end()
        break
    cv2.waitKey(1)

# Clean up
cv2.destroyAllWindows()