from djitellopy import tello

import KeyPressModule as kp

import numpy as np

from time import sleep


import math


nuevo = str(input("El mapeado de obstaculos en nuevo? (SI/NO): "))
if nuevo == "SI":
    pos = (250, 250)
    obstaculos = []
    mapeado = np.zeros(500, 500, 3)
    ocuppancy_grid_map = np.zeros(250, 250, dtype=np.uint8)
    os.remove("obstaculos.txt")
    os.remove("mapeado.txt")
    os.remove("posicion.txt")
    np.savetxt('obstaculos.txt', obstaculos)
    np.savetxt('mapeado.txt', mapeado)
    np.savetxt('mapeado.txt', mapeado)
    print("Escriba las coordenadas (x,y) de destino en metros con precision de 0.1 m")
    x = float(input("Coordenada x = "))  # metros
    y = float(input("Coordenada y = "))
    destm = (x, y)
    destpx = convdist(destm)
elif nuevo == "NO":
    obstaculos = np.loadtxt('obstaculos.txt', dtype=int)
# luego el en loop: mapeado = np.loadtxt('mapeado.txt',dtype=int)
# cuando acabe     llamar desde el loop al final para tenerlo actualizao antes de resize si se da el caso
# todos os.remove("obstaculos.txt")
#     os.remove("mapeado.txt")
#     np.savetxt('obstaculos.txt', obstaculos)
#     np.savetxt('mapeado.txt', mapeado)
def get_dist_btw_pos(pos0, pos1):
    """
    Get distance between 2 mouse position.
    """
    x = abs(pos0[0] - pos1[0])
    y = abs(pos0[1] - pos1[1])
    dist_px = math.hypot(x, y)
    dist_cm = dist_px * MAP_SIZE_COEFF
    return int(dist_cm), int(dist_px)


def get_angle_btw_line(pos0, pos1, posref):
    """
    Get angle between two lines respective to 'posref'
    NOTE: using dot product calculation.
    """
    ax = posref[0] - pos0[0]
    ay = posref[1] - pos0[1]
    bx = posref[0] - pos1[0]
    by = posref[1] - pos1[1]
    # Get dot product of pos0 and pos1.
    _dot = (ax * bx) + (ay * by)
    # Get magnitude of pos0 and pos1.
    _magA = math.sqrt(ax**2 + ay**2)
    _magB = math.sqrt(bx**2 + by**2)
    _rad = math.acos(_dot / (_magA * _magB))
    # Angle in degrees.
    angle = (_rad * 180) / math.pi
    return int(angle)

def automatico(img, destino=None):
    if destino == None:
        print("Escriba las coordenadas (x,y) de destino en metros con precision de 0.2 m")
        x = float(input("Coordenada x = "))  # metros
        y = float(input("Coordenada y = "))
        x = int(x*30 + 1500)  # posicion pixeles
        y = int(1500 - y*30)
        destino = (x, y)
        path = [(0, 0)]
    return destino, path