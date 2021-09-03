
import KeyPressModule as kp
import Manual

import numpy as np

import time

import cv2
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM

# Valores iniales del drone
x, y = 250, 250  # posicion inicial
yaw = 0 # angulo inicial
OBSTACLE = 255
UNOCCUPIED = 0

x_dim = 500
y_dim = 500
start = (x, y)
destpx = (500, 500)
view_range = 5
map = OccupancyGridMap(x_dim, y_dim, '8N')
kp.init()
points = [(0, 0), (0, 0)]
modo = 0
obstaculos = []
for j in range(10):
    for p in range(-3, 3):
        map.set_obstacle((225 + 10 - j + p, 225 + 5 + j))
        obstaculos.append((225 + 10 - j + p, 225 + 5 + j))
for j in range(5):
        map.set_obstacle((j, j))
        obstaculos.append((j , j))
i = 0
pos = (250, 250)
lastpos = (x-1, y-1)
slam = SLAM(map=map, view_range=view_range)
new_observation = {"pos": None, "type": None}
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

def drawpoints(img, points, pos, obstaculos, angulo=0.0, modo = 0):
    global path
    for point in points:

        cv2.circle(img, point, 1, (240, 240, 240), cv2.FILLED)  # bgr color

    for point in obstaculos:

        cv2.circle(img, point, 1, (10, 130, 240), cv2.FILLED)
        # bgr color

    cv2.putText(img, f'({round((points[-1][0] - 250)/10, 2)},{round((-points[-1][1] + 250)/10, 2)}) m {angulo}gr',
                (points[-1][0] + 3, points[-1][1] + 5), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 0, 255), 1)
    if modo == 1:
        for point in path:
            cv2.circle(img, point, 1, (130, 130, 240), cv2.FILLED)
    cv2.drawMarker(img, pos, (255, 0, 0), cv2.MARKER_STAR, 6, 1)


while True:
    mapeado = np.zeros((500, 500, 3), np.uint8)
    if kp.getKey('m'):
        if modo == 0:
            path = []
            destm = []
            destpx = []
            i = 0
        modo = not modo

    if kp.getKey('r'):
        points = [(0, 0), (0, 0)]

    if kp.getKey('g'):
        if modo == 1:
            i = 0
        time.sleep(0.2)


    if modo == 0:
        [vals, pos, yaw] = Manual.getkeyboardinput()
        time.sleep(0.2)

    elif modo == 1:
        if i == 0:

            print("Escriba las coordenadas (x,y) de destino en metros con precision de 2 decimales")
            x = float(input("Coordenada x = "))  # metros
            y = float(input("Coordenada y = "))
            destm = (x, y)
            destpx = convdist(destm)

            slam = SLAM(map=map, view_range=view_range)
            new_observation = {"pos": None, "type": None}
            dstar = DStarLite(map, pos, destpx)
            path, g, rhs = dstar.move_and_replan(robot_position=pos)
            c = len(path)

        if new_observation is not None:
            old_map = map
            slam.set_ground_truth_map(gt_map=map)


        if pos != lastpos:
            lastpos = pos

            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=pos)
            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

            # d star
            path, g, rhs = dstar.move_and_replan(robot_position=pos)
            c2 = len(path)
            print(path)
        # pf.replan()
        # path=pf.get_path()
        cv2.drawMarker(mapeado, destpx, (150, 230, 150), cv2.MARKER_DIAMOND, 6, 1)
        cv2.putText(mapeado, f'({round(destm[0] , 2)},{round(destm[1] , 2)},) m ',
                    (destpx[0] + 5, destpx[1] + 10), cv2.FONT_HERSHEY_PLAIN, 0.75, (150, 230, 150), 1)
        i = i + 1
        if i % 50 == 0:
            obstaculos = np.unique(obstaculos, axis=0)
        lastpos = pos
        print(i)

        if len(path) == 1 or 0:
            print("Ha llegado a su destino, aterrice o pulse la tecla G para seleccionar un nuevo destino")
        else:
            pos = path[1]
        time.sleep(0.25)
    if points[-1][0] != pos[0] or points[-1][1] != pos[1]:
        points.append(pos)
    # Mapeado
    drawpoints(mapeado, points, pos, obstaculos, yaw, modo)
    cv2.imshow('Mapeado', mapeado)

    if kp.getKey('q'):
        cv2.imwrite(f'Mapeado/mapeadofinal{time.time()}.jpg', mapeado)
        time.sleep(0.2)
        print('¡Hasta la proxima')
        break

    cv2.waitKey(1)

# Clean up
cv2.destroyAllWindows()
