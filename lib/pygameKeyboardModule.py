import pygame

def get_input():
    pressed = pygame.key.get_pressed()
    q = pressed[pygame.K_q]
    w = pressed[pygame.K_w]
    e = pressed[pygame.K_e]
    r = pressed[pygame.K_r]
    a = pressed[pygame.K_a]
    s = pressed[pygame.K_s]
    d = pressed[pygame.K_d]
    f = pressed[pygame.K_f]
    esc = pressed[pygame.K_ESCAPE]
    return q, w, e, r, a, s, d, f, esc

pygame.init()
size = width, height = 320, 240
screen = pygame.display.set_mode(size)
while True:
    pygame.event.pump()
    print(get_input())