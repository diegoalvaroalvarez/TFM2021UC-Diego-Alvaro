import math
import numpy as np
def angulodegiro(pos0, pos1, posref):
    """
    Obtener el angulo de giro entre dos puntos respecto a un punto de referencia
    Devuelve el angulo entre 0-180 y el sentido horario-antihorario
    """
    ax = posref[0] - pos0[0]  # a vector pos0aposderef
    ay = posref[1] - pos0[1]
    print(ax, ay)
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

pos = (250, 0)
yaw = 0
points = [(0, 0 )]
obstaculos = [(0, 0, 4), (0, 0, 4)]
yaw=np.load('datos/angulo.npy')
pos = np.load('datos/posicion.npy')
obs=np.load('datos/obstaculos.npy')
print(yaw, pos,obs)
a, giro=angulodegiro((250,250),(251,250),(250,250))
print(a, giro)
