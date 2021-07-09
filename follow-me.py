# Use W, A, S, D for moving, E, Q for rotating and R, F for going up and down.
# When starting the script the Tello will takeoff, pressing ESC makes it land
#  and the script exit.

from djitellopy import Tello
from threading import Thread
import pygame
import cv2, math, time
import mediapipe as mp
from lib.helper import get_bb

LINEAR_SPEED = 50
YAW_SPEED = 25
DATA_STREAM = True

SHORT_RANGE = False
DETECTION_CONFIDENCE = 0.8

FOLLOW_ME = True

def videoStreamer():
    mp_face_detection = mp.solutions.face_detection
    while DATA_STREAM:
        with mp_face_detection.FaceDetection(
                model_selection=0 if SHORT_RANGE else 1, 
                min_detection_confidence=DETECTION_CONFIDENCE) as face_detection:
            img = frame_read.frame
            draw = img.copy()

            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
            draw = cv2.cvtColor(cv2.flip(draw, 1), cv2.COLOR_BGR2RGB)
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            draw.flags.writeable = False
            results = face_detection.process(draw)

            # Draw the face mesh annotations on the image.
            draw.flags.writeable = True
            draw = cv2.cvtColor(draw, cv2.COLOR_RGB2BGR)
            if results.detections:
                for detection in results.detections:
                    cx_min, cy_min, cx_max, cy_max = get_bb(draw, detection)
                    if cx_min is not None:
                        cv2.rectangle(draw, (cx_min, cy_min), (cx_max, cy_max), (0, 255, 0), 2)
            draw = cv2.resize(draw, (320,240))

        cv2.imshow("drone", draw)
        cv2.waitKey(1)
    cv2.destroyAllWindows()

def get_input():
    pygame.event.pump()
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

def getVelocityFromKeyboard():
    lr, fb, ud, yaw = 0, 0, 0, 0
    
    q, w, e, r, a, s, d, f, esc = get_input()

    if a: lr = -LINEAR_SPEED
    elif d: lr = LINEAR_SPEED

    if w: fb = LINEAR_SPEED
    elif s: fb = -LINEAR_SPEED

    if r: ud = LINEAR_SPEED
    elif f: ud = -LINEAR_SPEED

    if q: yaw = -YAW_SPEED
    elif e: yaw = YAW_SPEED

    return lr, fb, ud, yaw, esc

def init():
    pygame.init()
    size = width, height = 100,100
    pygame.display.set_mode(size)

tello = Tello()
tello.connect()
init()

tello.streamon()
frame_read = tello.get_frame_read()

streamer = Thread(target=videoStreamer)
streamer.start()

tello.takeoff()

while True:
    lr, fb, ud, yaw, isEsc = getVelocityFromKeyboard()
    if not isEsc:
        tello.send_rc_control(lr, fb, ud, yaw)
    else:
        break

tello.land()
DATA_STREAM = False
streamer.join()
