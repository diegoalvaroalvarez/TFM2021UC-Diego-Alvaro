# TFM2021UC-Diego-Alvaro
Archivos utilizados para mi Proyecto Fin de Máster de Ingeniería Industrial en la Universidad de Cantabria.


TÍTULO: Planificación de trayectorias para el control autónomo de un cuadricóptero utilizando técnicas de visión artificial y aprendizaje profundo

TITLE: Path planning for the autonomous control of a quadropter aplying artifitial vision techniques and Deep Learning

Resumen:

En este trabajo se abordará la problemática de la planificación de trayectorias y evasión de obstáculos para vehículos autónomos no tripulados (UAV = unmanned aerial vehicle) en entornos dinámicos, entre otras aplicaciones se emplea en transporte, mensajería, logística y operaciones de rescate.
Después de describir las herramientas y técnicas utilizadas, se desarrollará un prototipo sobre un cuadricóptero o drone (DJI Tello) que se programará para ser controlado manualmente mediante una interfaz de usuario desde un ordenador o poder moverse de forma autónoma introduciéndole las coordenadas del destino deseado, mientras construye un mapeado del entorno, registrando la trayectoria del drone y los obstáculos encontrados, así como la estimación de la posición y orientación de la aeronave mediante odometría.
Para la detección de obstáculos se implementará un modelo entrenado con aprendizaje profundo (Deep Learning) para reconocer en tiempo real, a partir de las imágenes que proporciona la cámara, distintos tipos de obstáculos como árboles, vallas, personas…
Para el movimiento autónomo se utilizará un algoritmo de distancia mínima para crear una ruta segura hasta el destino deseado evadiendo los obstáculos conocidos. Por tanto, cada vez que se encuentra un obstáculo nuevo, se actualizará el mapeado y, si es necesario, la ruta óptica hasta la localización objetivo.

Abstract:

This project discusses the problematic of path planning and obstacle avoidance in unmanned autonomous vehicles (UAV) in dynamic environments in different application fields such as transportation, messenger service, logistics and rescue operations.
After describing what tools and techniques have been used, it will develop a prototype of a quadcopter or drone (DJI Tello) which will be programmed to be able to be controlled manually by an interface or to move autonomously just introducing the coordinates of a goal destination, while it is mapping the environment and keeping track of the path and obstacles. Also, it will estimate the current position and orientation of the aerial robot applying odometry techniques.
For the object detection task, it will use a model which has been trained applying Deep Learning techniques for recognizing in real time different types of obstacles such as trees, walls, people, etc. in images provided by the DJI Tello camera.
For the autonomous movement task, it will use a shortest path algorithm to elaborate a safe path to the goal destination avoiding known obstacles. Hence, every time it detects a new obstacle, it will map it and, if it is necessary, it will recalculate the optimal path to the goal location.
