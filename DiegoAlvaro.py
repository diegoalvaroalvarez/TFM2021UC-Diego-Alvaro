from djitellopy import tello
import KeyPressModule as kp
import Manual
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
    if ax != -ay:
        py = -ax / (ax+ay)  # p vector perpendicular a vector a
        px = py + 1
    else:
        py = ax/ay
        px = 2
    bx = posref[0] - pos1[0] # b vector de posrefapos1
    by = posref[1] - pos1[1]
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
    elif angle2 <= 90:
        sentido = 'antihorario'
        angle = 180 - angle
    else:
        sentido = 'horario'
        angle = 180 - angle

    return int(angle), sentido

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
        elif obstaculos[w][2] == 8:  # Edificio
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]),  (0, 128, 255), cv2.MARKER_DIAMOND, 4, 2)   # naranja
        elif obstaculos[w][2] == 6:  # Semaforo
            cv2.drawMarker(img, (obstaculos[w][0], obstaculos[w][1]), (155, 155, 155), cv2.MARKER_TILTED_CROSS, 4, 1)  # gris

def drawpoints(img, points, pos, angulo=0.0, modo = 0):
    global path
    for point in points:

        cv2.circle(img, point, 1, (240, 240, 240), cv2.FILLED)  # bgr color circulos blancos


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
PATH_TO_LABELS = os.path.join(CWD_PATH, 'training', 'labelmap.pbtxt')
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
nuevo = str(input("El mapeado de obstaculos en nuevo? (SI/NO): "))
if nuevo == "SI": #habrá que meter lo del gridmap si tal o que
    pos = (250, 250)
    obstaculos = [(0, 0, 4), (0, 0, 4)]  # los dos primeros no se toman en cuenta y es para no repetir obstaculos
    mapeado = np.zeros((500, 500, 3), np.uint8)
    yaw = 0
    points = [(0, 0)]

elif nuevo == "NO":
    obstaculos = np.load('datos/obstaculos.npy')
    pos = np.load('datos/posicion.npy')
    yaw = np.load('datos/angulo.npy')
    points = np.load('datos/recorrido.npy')

print("Escriba las coordenadas (x,y) de destino en metros con precision de 0.1 m")
x = float(input("Coordenada x = "))  # metros
y = float(input("Coordenada y = "))
destm = (x, y)
destpx = convdist(destm)
dstar = DStarLite(map, pos, destpx)
path, g, rhs = dstar.move_and_replan(robot_position=pos)

kp.init()

me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()

while True:
    mapeado = np.zeros((500, 500, 3), np.uint8)
    if kp.getKey('l'):
        me.land()
        time.sleep(2)
    if kp.getKey('t'):
        me.takeoff()
    if kp.getKey('m'):
        if modo == 0:
            i = 0
        modo = not modo
    if kp.getKey('r'):
        points = [(0, 0), (0, 0)]
    if kp.getKey('g'):  # nuevo destino
        # points = [(0, 0), (0, 0)]
        if modo == 1:
            i = 0

    if kp.getKey('f'):
        cv2.imwrite(f'Fotografias/fotografiadrone{time.time()}.jpg', frame)
        time.sleep(0.2)
    if kp.getKey('b'):
        print(me.get_battery())
    if kp.getKey('q'):
        cv2.imwrite(f'Mapeado/mapeadofinal{time.time()}.jpg', mapeado)
        time.sleep(0.2)
        np.save('datos/posicion.npy', pos)
        np.save('datos/angulo.npy', yaw)
        np.save('datos/recorrido.npy', points)
        np.save('datos/obstaculos.npy', obstaculos)
        print('¡Hasta la proxima')
        me.end()
        break

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

    if len(delante) > 0 and (pos[0] + round(20*math.sin(yaw)), pos[1] - round(20*math.cos(yaw)), delante[0]) != obstaculos[-len(delante)]:
        for j in range(-1, 1):
            for p in range(-1, 1):
                map.set_obstacle((pos[0] + round(20*math.sin(yaw)) + p, pos[1] - round(20*math.cos(yaw)) + j))
        for c in range(len(delante)):
            obstaculos.append((pos[0] + round(20*math.sin(yaw)), pos[1] - round(20*math.cos(yaw)), delante[c]))

    if modo == manual:
        if pos != lastpos:
            lastpos = pos
        [vals, pos, yaw] = Manual.getkeyboardinput()
        me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        time.sleep(0.25)

    elif modo == automatico:
        """
        if i == 0:
            print("Escriba las coordenadas (x,y) de destino en metros con precision de 0.1 m")
            x = float(input("Coordenada x = "))  # metros
            y = float(input("Coordenada y = "))
            destm = (x, y)
            destpx = convdist(destm)
            for j in range(10):
                for p in range(-3, 3):
                    map.set_obstacle((255+10-j+p, 255+5+j))
                    obstaculos.append((255+10-j+p, 255+5+j))

            dstar = DStarLite(map, pos, destpx)
            path, g, rhs = dstar.move_and_replan(robot_position=pos)
            c = len(path)
        """
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
                print(path)

            i = i + 1
            if i % 50 == 0:
                obstaculos = np.unique(obstaculos, axis=0)

            print(i)

            if len(path) == 1 or 0:
                print("Ha llegado a su destino, aterrice o pulse la tecla G para seleccionar un nuevo destino")
                me.send_rc_control(0, 0, 0, 0)
                time.sleep(0.25)
            else:
                pos = path[1]
                nextpos = destpx
                if len(path) > 2:
                    nextpos = path[2]
                anggiro , angsent = angulodegiro(lastpos, nextpos, pos)
                if anggiro != 0:
                    if angsent == 'horario':
                        me.rotate_clockwise(anggiro)
                    else:
                        me.rotate_counter_clockwise(anggiro)
                    yaw = yaw + anggiro
                    if yaw > 180:
                        yaw = yaw - 360 * (yaw // 180)
                    elif yaw < -180:
                        yaw = yaw + 360 * (-yaw // 180)

                me.send_rc_control(0, 30, 0, 0)
                time.sleep(0.25)

    if points[-1][0] != pos[0] or points[-1][1] != pos[1]:
        points.append(pos)
    # Mapeado
    print(obstaculos)
    cv2.drawMarker(mapeado, destpx, (150, 230, 150), cv2.MARKER_DIAMOND, 6, 1) # rombo verde
    cv2.putText(mapeado, f'({round(destm[0], 2)},{round(destm[1], 2)},) m ',
                (destpx[0] + 5, destpx[1] + 10), cv2.FONT_HERSHEY_PLAIN, 0.75, (150, 230, 150), 1)
    drawobstacles(mapeado, obstaculos)
    drawpoints(mapeado, points, pos, yaw, modo)
    cv2.imshow('Mapeado', mapeado)
    # Camara drone
    img = cv2.resize(frame, (480, 360))
    cv2.imshow("DJI TELLO", img)

    cv2.waitKey(1)

# Clean up
cv2.destroyAllWindows()
