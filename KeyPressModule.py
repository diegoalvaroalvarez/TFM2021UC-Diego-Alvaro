import pygame
import time


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

def main():
    if getKey('LEFT'):

        print('Left key pressed')

    if getKey('RIGHT'):
        print('Right key Pressed')

    if getKey('l'):
        me.land()
        time.sleep(2)
    if getKey('t'):
        me.takeoff()
    if getKey('m'):
        modo = not modo
    if getKey('r'):
        points = [(0, 0), (0, 0)]
        destino = None
    if getKey('g'):
        cv2.imwrite(f'Mapeado/mapeado{time.time()}.jpg', mapeado)
        time.sleep(0.2)
    if getKey('f'):
        cv2.imwrite(f'Fotografias/fotografiadrone{time.time()}.jpg', frame)
        time.sleep(0.2)
    if getKey('b'):
        print(me.get_battery())

if __name__ == '__main__':

    init()

    while True:

        main()