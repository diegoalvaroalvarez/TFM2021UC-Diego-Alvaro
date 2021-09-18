from djitellopy import tello
import Manual0 as man
import math
import numpy as np
import time
import cv2


# Valores iniales del drone
x, y = 250, 250  # posicion inicial
yaw = 0  # angulo inicial
x_dim = 500
y_dim = 500
vals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
points = [(0, 0), (0, 0)]
obstaculos2 = []
i = 0
pos = (x, y)
lastpos = (x-1, y-1)
modo = False
manual = False
automatico = True

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
    if num > 0:
        return 1
    else:
        return 0



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
# me.streamon()

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

    if man.getKey('b'):
        print(me.get_battery())
        time.sleep(2)


    # frame = me.get_frame_read().frame

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

                if anggiro != 0:

                    if angsent == 'horario':
                        yaw = yaw + anggiro
                        me.send_rc_control(0, 0, 0, 100)
                        time.sleep(anggiro/100+0.1)
                        me.send_rc_control(0, 30, 0, 0)
                        time.sleep(0.35)

                    else:
                        yaw = yaw - anggiro
                        me.send_rc_control(0, 0, 0, -100)
                        time.sleep(anggiro/100+0.1)
                        me.send_rc_control(0, 30, 0, 0)
                        time.sleep(0.35)

                    if yaw > 180:
                        yaw = yaw - 360 * (yaw // 180)
                    elif yaw < -180:
                        yaw = yaw + 360 * (-yaw // 180)
                else:
                    me.send_rc_control(0, 30, 0, 0)
                    time.sleep(0.3)
                me.send_rc_control(0, 0, 0, 0)
                time.sleep(0.25)

    if points[-1][0] != pos[0] or points[-1][1] != pos[1]:
        points.append(pos)

    drawpoints(mapeado, points, pos, yaw, modo)
    cv2.imshow('Mapping', mapeado)
    # Camara drone
    # img = cv2.resize(frame, (480, 360))
    # cv2.imshow("DJI TELLO", img)
    # time.sleep(0.1)
    if man.getKey('q'):
        print(me.get_distance_tof())
        print('¡Hasta la proxima')
        me.end()
        break
    cv2.waitKey(1)

# Clean up
cv2.destroyAllWindows()