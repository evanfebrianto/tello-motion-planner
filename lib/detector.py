import cv2
import time
import numpy as np
import mediapipe as mp
from lib.helper import get_bb, calculateDistance

class Detector():
    def __init__(self, 
                tello_instance=None,
                width = 1280,
                height = 720,
                distance_thresh = 40,
                bb_thresh = 100,
                pid_x=None,
                pid_y=None,
                pid_z=None,
                pid_yaw=None) -> None:

        # Tello initilization
        self.TIME_BTW_RC_CONTROL_COMMANDS = 0.15
        self.tello = tello_instance
        self.frame_read = self.tello.get_frame_read()
        self.img_draw = None
        self.isFailed = False
        self.isLandingCondition = True
        self.distance = float('inf')
        self.dist_thresh = distance_thresh
        self.bb_thresh = bb_thresh

        # Camera Initialization
        self.width, self.height = width, height

        # Thread mark initialization
        self.isThreadActive = True

        # Detection Initialization
        self.face = None # will contain tuple xc, yc, dx
        self.thumb_tip = None # will save the x and y coordinate of thumb tip

        # PID initialization
        self.pid_x = pid_x
        self.pid_y = pid_y
        self.pid_z = pid_z
        self.pid_yaw = pid_yaw

        # speed control
        self.lr_vel = 0
        self.fb_vel = 0
        self.ud_vel = 0
        self.yaw_vel = 0


    def faceHandDetector(self) -> None:
        mp_drawing = mp.solutions.drawing_utils
        mp_face_detection = mp.solutions.face_detection
        mp_hands = mp.solutions.hands
        con_drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1, color=(204, 194, 0))
        landmark_drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1, color=(167, 129, 0))
        thumb_x, thumb_y, cx_min, cy = None, None, None, None
        while self.isThreadActive:
            img = self.frame_read.frame.copy()
            draw = cv2.resize(img, (self.width,self.height))
            self.face = None
            self.thumb_tip = None
            self.distance = float('inf')
            with mp_hands.Hands(
                max_num_hands=1,
                min_detection_confidence=0.3,
                min_tracking_confidence=0.5) as hands:
                with mp_face_detection.FaceDetection(
                        model_selection=1, 
                        min_detection_confidence=0.7) as face_detection:
                    # Flip the image horizontally for a later selfie-view display, and convert
                    # the BGR image to RGB.b
                    draw = cv2.cvtColor(cv2.flip(draw, 1), cv2.COLOR_BGR2RGB)
                    # To improve performance, optionally mark the image as not writeable to
                    # pass by reference.
                    draw.flags.writeable = False
                    results_hand = hands.process(draw)
                    results_face = face_detection.process(draw)

                    # Draw the face mesh annotations on the image.
                    draw.flags.writeable = True
                    draw = cv2.cvtColor(draw, cv2.COLOR_RGB2BGR)

                    if results_hand.multi_hand_landmarks:
                        for hand_landmarks in results_hand.multi_hand_landmarks:
                            thumb_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x * self.width
                            thumb_y = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y * self.height
                            self.thumb_tip = thumb_x, thumb_y
                            mp_drawing.draw_landmarks(
                                draw, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                                connection_drawing_spec=con_drawing_spec,
                                landmark_drawing_spec=landmark_drawing_spec)

                    if results_face.detections:
                        for detection in results_face.detections:
                            cx_min, cy_min, cx_max, cy_max = get_bb(draw, detection)
                            if cx_min is not None:
                                dx, dy = (cx_max-cx_min, cy_max-cy_min)
                                cx = cx_min + dx//2
                                cy = cy_min + dy//2
                                self.face = cx, cy, dx
                                cv2.rectangle(draw, (cx_min, cy_min), (cx_max, cy_max), (0, 255, 0), 2)

                    if self.face is not None and self.thumb_tip is not None:
                        self.distance = calculateDistance(self.face[0], self.face[1], self.thumb_tip[0], self.thumb_tip[1])
                        if self.distance < self.dist_thresh:
                            self.isLandingCondition = True
            self.img_draw = draw


    def videoStreamer(self) -> None:
        cv2.namedWindow('drone', cv2.WINDOW_NORMAL)
        while self.isThreadActive:
            img = self.frame_read.frame.copy()
            img = cv2.resize(img, (self.width,self.height))
            if self.img_draw is not np.empty:
                cv2.imshow("drone", self.img_draw)
            else:
                cv2.imshow("drone", img)
            cv2.waitKey(1)
        cv2.destroyAllWindows()


    def videoRecorder(self):
        # create a VideoWrite object
        FPS = 25
        video = cv2.VideoWriter('face_tracking.mp4',cv2.VideoWriter_fourcc('M','J','P','G'), FPS, (self.width,self.height))
        while self.isThreadActive:
            img = self.frame_read.frame.copy()
            img = cv2.resize(img, (self.width, self.height))
            if self.img_draw is not np.empty:
                try:
                    self.img_draw = cv2.resize(self.img_draw, (self.width, self.height))
                    video.write(self.img_draw)
                except:
                    pass
            time.sleep(1 / FPS)
        video.release()
    

    def controller(self) -> None:
        def clampSpeed(x:float, maxSpeed=50) -> int:
            return max(-maxSpeed, min(maxSpeed, int(x)))

        while self.isThreadActive:
            if self.face is None:
                self.lr_vel = 0
                self.fb_vel = 0
                self.ud_vel = 0
                self.yaw_vel = 0
            else:
                # Define reference point
                # x, y, z
                _ref = self.width//2, self.height//2, self.bb_thresh
                
                # Define cross track error
                _cte = _ref - self.face
                
                # Update error
                self.pid_x.updateError(_cte[0])
                self.pid_y.updateError(_cte[1])
                self.pid_z.updateError(_cte[2])
                self.pid_yaw.updateError(_cte[0])

                # Calculate the error and set the speed accordingly
                self.lr_vel = clampSpeed(self.pid_x.totalError())
                self.ud_vel = clampSpeed(self.pid_y.totalError())
                self.fb_vel = clampSpeed(self.pid_z.totalError())
                self.yaw_vel = clampSpeed(self.pid_yaw.totalError())
            
            if self.isLandingCondition:
                self.lr_vel = 0
                self.fb_vel = 0
                self.ud_vel = 0
                self.yaw_vel = 0

            self.tello.send_rc_control(
                        self.lr_vel, # left_right_velocity
                        self.fb_vel, # for_back_velocity
                        self.ud_vel, # up_down_velocity
                        self.yaw_vel) # yaw_velocity
            time.sleep(self.TIME_BTW_RC_CONTROL_COMMANDS)

    def takeoff(self):
        try:
            self.tello.takeoff()
            time.sleep(1)
        except:
            self.isFailed = True

    def land(self):
        try:
            self.tello.land()
        except:
            pass

    def end(self):
        self.tello.end()