from djitellopy import tello
import Manual0 as man
import math
import os
import numpy as np
import sys
import time
import tensorflow as tf
import cv2

from grid import OccupancyGridMap, SLAM

# Valores iniales del drone
p0=[(0,0)]
p1=(0, 0, 4), (0, 0, 5)
OBSTACLE = 255
UNOCCUPIED = 0
x_dim = 500
y_dim = 500

view_range = 5
map = OccupancyGridMap(x_dim, y_dim, '8N')
vals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
points = [(0, 0), (0, 0)]

i = 0
k = 0

slam = SLAM(map=map, view_range=view_range)
new_observation = {"pos": None, "type": None}
modo = False
manual = False
automatico = True
delante = []

def convdist(posicion, modo=0):
    """
    :param posicion: Coordenadas de un punto del mapa
    :param modo: si 0 pasa de m a px, si 1 pasa de px a m
    :return: Coordenadas convertidas
    """
    nposicion = []
    if modo == 0:
        nposicion.append(int(posicion[0]*100/10+250))
        nposicion.append(int(round(250-posicion[1] * 100 / 10)))

    elif modo == 1:
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
        py = -ax/ay
        px = 1

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
    elif angle2 == 0 or 90<angle2<180:
        sentido = 'antihorario'
        angle = angle
    elif 0 < angle2 <= 90 or angle2==180 or (angle2==0 and (bx<0 or by<0)):
        sentido = 'horario'
        angle = angle

    print(angle,sentido)
    return int(angle), sentido

def signo(num):
    if num > 0:
        return 1
    else:
        return 0

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
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]), (155, 155, 155), cv2.MARKER_TILTED_CROSS, 4, 1)  # gris  ,

def drawpoints(img, points, pos, angulo=0.0, modo = 0):
    global path, destpx, destm
    for point in points:

        cv2.circle(img, point, 1, (240, 240, 240), cv2.FILLED)  # bgr color circulos blancos

    cv2.drawMarker(mapeado, destpx, (150, 230, 150), cv2.MARKER_DIAMOND, 6, 1) # rombo verde
    cv2.putText(mapeado, f'({round(destm[0], 2)},{round(destm[1], 2)},) m ',
                (destpx[0] + 5, destpx[1] + 10), cv2.FONT_HERSHEY_PLAIN, 0.75, (150, 230, 150), 1)

    cv2.putText(img, f'({round((points[-1][0] - 250)/10, 2)},{round((-points[-1][1] + 250)/10, 2)},{me.get_height()/100}) m {angulo}gr',
                (points[-1][0] + 3, points[-1][1] + 5), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 0, 255), 1)
    if modo == 1:
        for point in path:
            cv2.circle(img, point, 1, (130, 130, 240), cv2.FILLED) #circulos rojo claro
    cv2.drawMarker(img, pos, (255, 0, 0), cv2.MARKER_STAR, 6, 1) # Estrella azul

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

# Name of the directory containing the object detection module we're using
MODEL_NAME = 'new_graph'
# Grab path to current working directory
CWD_PATH = os.getcwd()
# Path to frozen detection graph .pb file, which contains the model that is used for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')
# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH, 'Preparation', 'labelmap.pbtxt')
# Number of classes the object detector can identify
NUM_CLASSES = 8
# Load the label map.
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
category_index = label_map_util.create_category_index(categories)
# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.compat.v1.Session(graph=detection_graph)


# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')
# Comienzo del programa
nuevo = str(input("Reiniciar datos de vuelo? (SI/NO): "))
if nuevo == "SI":
    pos = (250, 250)
    obstaculos = [(0, 0, 4), (0, 0, 5)]  # los dos primeros no se toman en cuenta y es para no repetir obstaculos
    yaw = 0
    lastpos =(250, 249)
    points = [(0, 0)]
    np.save('datos/obs.npy', 0)

else:
    obstaculos = np.load('datos/obstaculos.npy')
    for y in range(0,len(obstaculos)):
        p1.append((obstaculos[y][0], obstaculos[y][1]),obstaculos[y][2])
    obstaculos = p1
    pos = np.load('datos/posicion.npy')
    x = pos[0]
    y = pos[1]
    pos = (x, y)
    yaw = np.load('datos/angulo.npy')
    points = np.load('datos/recorrido.npy')
    for z in range(len(points)):
        p0.append((points[z][0], points[z][1]))
    points = p0
    lastpos = np.load('datos/lastposicion.npy')
    lastpos=(lastpos[0], lastpos[1])
    np.save('datos/obs.npy', 1)

from d_star_lite import DStarLite

print("Escriba las coordenadas (x,y) de destino en metros con precision de 0.1 m")
dx = float(input("Coordenada x = "))
dy = float(input("Coordenada y = "))
destm = (dx, dy)
destpx = convdist(destm)
dstar = DStarLite(map, pos, destpx)
path, g, rhs = dstar.move_and_replan(robot_position=pos)

man.init()

me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()

while True:


    if man.getKey('l'):
        me.land()
        time.sleep(2)
    elif man.getKey('t'):
        me.takeoff()
        time.sleep(1)
    if man.getKey('m'):
        print('Modo cambiado!')
        if modo == 0:
            i = 0
        modo = not modo
        time.sleep(0.25)
    elif man.getKey('r'):
        points = [(0, 0), (0, 0)]

    if man.getKey('f'):
        cv2.imwrite(f'Fotografias/fotografiadrone{time.time()}.jpg', frame)
        time.sleep(0.2)
    elif man.getKey('b'):
        print(me.get_battery())
    if man.getKey('q'):
        print(me.get_distance_tof())
        cv2.imwrite(f'Mapeado/mapeadofinal{time.time()}.jpg', mapeado)
        time.sleep(0.1)
        obstaculos = np.unique(obstaculos, axis=0)
        for j in range(2, len(obstaculos)):
            if obstaculos[j-k][2]==[5] or obstaculos[j-k][2]==[7]:
                del obstaculos[j-k]
                k+=1
        np.save('datos/posicion.npy', pos)
        np.save('datos/lastposicion.npy', lastpos)
        np.save('datos/angulo.npy', yaw)
        np.save('datos/recorrido.npy', points)
        np.save('datos/obstaculos.npy', obstaculos)
        print('¡Hasta la proxima')
        me.end()
        time.sleep(1)
        break

    mapeado = np.zeros((500, 500, 3), np.uint8)

    # Deteccion de objetos con el drone
    frame = me.get_frame_read().frame
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_expanded = np.expand_dims(frame_rgb, axis=0)
     # Perform the actual detection by running the model with the image as input

    (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})
    # Draw the results of the detection (aka 'visulaize the results')
    [frame, delante] = vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=3,
            min_score_thresh=0.85)
    # Definir obstáculos nuevos en la lista de obstáculos
    if len(delante) > 0 and (pos[0] + round(20*math.sin(yaw)), pos[1] - round(20*math.cos(yaw)), delante[0]) != obstaculos[-len(delante)]:
        for j in range(-3, 3):
            for p in range(-3, 3):
                map.set_obstacle((pos[0] + round(20*math.sin(yaw)) + p, pos[1] - round(20*math.cos(yaw)) + j))
        for c in range(len(delante)):
            obstaculos.append((pos[0] + round(20*math.sin(yaw)), pos[1] - round(20*math.cos(yaw)), delante[c]))

    if modo == manual:
        if pos != lastpos:
            lastpos = pos
        [vals, pos, yaw] = man.getkeyboardinput()
        me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        time.sleep(0.25)

    elif modo == automatico:

        if i == 0:
            path, g, rhs = dstar.move_and_replan(robot_position=pos)
            nextpos = path[1]
            anggiro, angsent = angulodegiro((pos[0]-1, pos[1]-1), path[1], pos)
            i=1
        else:
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
                # print(path)

            i = i + 1
            if i % 50 == 0:
                obstaculos = np.unique(obstaculos, axis=0)

            print(i)

            if len(path) == 1 or 0:
                print("Ha llegado a su destino, aterrice")
                me.send_rc_control(0, 0, 0, 0)
                time.sleep(0.25)
            else:
                pos = path[1]
                nextpos = destpx
                if len(path) > 2:
                    nextpos = path[2]


                anggiro, angsent = angulodegiro(lastpos, nextpos, pos)

                if anggiro != 0:

                    if angsent == 'horario':
                        yaw = yaw + anggiro
                        me.send_rc_control(0, 0, 0, 100)
                        time.sleep(anggiro/100)
                        me.send_rc_control(0, 30, 0, 0)
                        time.sleep(0.45)

                    else:
                        yaw = yaw - anggiro
                        me.send_rc_control(0, 0, 0, -100)
                        time.sleep(anggiro/100)
                        me.send_rc_control(0, 30, 0, 0)
                        time.sleep(0.45)

                    if yaw > 180:
                        yaw = yaw - 360 * (yaw // 180)
                    elif yaw < -180:
                        yaw = yaw + 360 * (-yaw // 180)
                else:
                    me.send_rc_control(0, 30, 0, 0)
                    time.sleep(0.33)
                me.send_rc_control(0, 0, 0, 0)
                time.sleep(0.2)

    if points[-1][0] != pos[0] or points[-1][1] != pos[1]:
        points.append(pos)
    # Mapeado
    #print(obstaculos)

    drawobstacles(mapeado, obstaculos)
    drawpoints(mapeado, points, pos, yaw, modo)
    cv2.imshow('Mapping', mapeado)
    # Camara drone
    frame = cv2.resize(frame, (480, 360))
    cv2.imshow("DJI TELLO", frame)

    cv2.waitKey(1)

# Clean up
cv2.destroyAllWindows()
