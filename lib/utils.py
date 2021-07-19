from djitellopy import Tello
import cv2
import numpy as np
import mediapipe as mp
from lib.helper import get_bb, calculateDistance
 
def initializeTello():
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0
    myDrone.left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0
    print(myDrone.get_battery())
    myDrone.streamoff()
    myDrone.streamon()
    return myDrone
 
def telloGetFrame(myDrone, w= 360,h=240):
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame,(w,h))
    return img
 
def findFace(img):
    mp_drawing = mp.solutions.drawing_utils
    mp_face_detection = mp.solutions.face_detection
    mp_hands = mp.solutions.hands
    con_drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1, color=(204, 194, 0))
    landmark_drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1, color=(167, 129, 0))
    thumb_x, thumb_y, cx_min, cy = None, None, None, None
    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.9,
        min_tracking_confidence=0.5) as hands:
        with mp_face_detection.FaceDetection(
                model_selection=1, 
                min_detection_confidence=0.7) as face_detection:
            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
            img = cv2.cvtColor(cv2.flip(img, 1), cv2.COLOR_BGR2RGB)
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            img.flags.writeable = False
            results_hand = hands.process(img)
            results_face = face_detection.process(img)
            image_height, image_width, _ = img.shape

            # Draw the face mesh annotations on the image.
            img.flags.writeable = True
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            myFaceListC = []
            myFaceListArea = []

            if results_hand.multi_hand_landmarks:
                for hand_landmarks in results_hand.multi_hand_landmarks:
                    thumb_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x * image_width
                    thumb_y = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y * image_height
                    mp_drawing.draw_landmarks(
                        img, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                        connection_drawing_spec=con_drawing_spec,
                        landmark_drawing_spec=landmark_drawing_spec)

            if results_face.detections:
                for detection in results_face.detections:
                    cx_min, cy_min, cx_max, cy_max = get_bb(img, detection)
                    if cx_min is not None:
                        dx, dy = (cx_max-cx_min, cy_max-cy_min)
                        cx = cx_min + dx//2
                        cy = cy_min + dy//2
                        area = dx * dy
                        myFaceListArea.append(area)
                        myFaceListC.append([cx,cy])
                        cv2.rectangle(img, (cx_min, cy_min), (cx_max, cy_max), (0, 255, 0), 2)
            if len(myFaceListArea) !=0:
                i = myFaceListArea.index(max(myFaceListArea))
                if thumb_x and thumb_y and cx_min and cy_max:
                    return img, [myFaceListC[i],myFaceListArea[i],calculateDistance(thumb_x,thumb_y, cx_max,cy)]
                else:
                    return img, [myFaceListC[i],myFaceListArea[i],float('inf')]
            else:
                return img,[[0,0],0,float('inf')]

def trackFace(myDrone,info,w,h,pid,pidA,pErrorX,pErrorY,pErrorA,last_errorX,last_errorY,last_errorA):
 
    ## PID
    error_x = info[0][0] - w//2
    error_y = info[0][1] - h//2
    if w==640:
        areaThreshold = 6400
    else:
        areaThreshold = 1800
    error_A = info[1] - areaThreshold
    d_errorX = error_x - last_errorX
    d_errorY = error_y - last_errorY
    d_errorA = error_A - last_errorA
    speed_yaw = pid[0]*error_x + pid[1]*(error_x-pErrorX) + d_errorX*pid[2]
    speed_up_down = pid[0]*error_y + pid[1]*(error_y-pErrorY) + d_errorY*pid[2]
    speed_for_back = pidA[0]*error_A + pidA[1]*(error_A-pErrorA) + d_errorA*pidA[2]
    speed_yaw = int(np.clip(speed_yaw,-100,100))
    speed_up_down = int(np.clip(speed_up_down,-100,100))
    speed_for_back = int(np.clip(speed_for_back,-70,70))
    last_errorX = error_x
    last_errorY = error_y
    last_errorA = error_A

    
    if info[0][0] !=0:
        myDrone.yaw_velocity = -speed_yaw
        myDrone.up_down_velocity = -speed_up_down
        myDrone.for_back_velocity = -speed_for_back
    else:
        myDrone.for_back_velocity = 0
        myDrone.left_right_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        error_x = 0
        error_y = 0
        error_A = 0
    if info[2] < 40.0:
        myDrone.for_back_velocity = 0
        myDrone.left_right_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        error_x = 0
        error_y = 0
        error_A = 0
    if myDrone.send_rc_control:
        myDrone.send_rc_control(myDrone.left_right_velocity,
                                myDrone.for_back_velocity,
                                myDrone.up_down_velocity,
                                myDrone.yaw_velocity)
    return error_x, error_y, error_A, last_errorX,last_errorY,last_errorA