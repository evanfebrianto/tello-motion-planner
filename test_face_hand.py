import cv2
import mediapipe as mp
from lib.helper import get_bb, calculateDistance

mp_drawing = mp.solutions.drawing_utils
mp_face_detection = mp.solutions.face_detection
mp_hands = mp.solutions.hands
con_drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1, color=(204, 194, 0))
landmark_drawing_spec = mp_drawing.DrawingSpec(thickness=2, circle_radius=3, color=(167, 129, 0))
thumb_x, thumb_y, cx_min, cy = None, None, None, None

# For webcam input:
cap = cv2.VideoCapture(0)
cap.set(3, 360)
cap.set(4, 240)
with mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5) as hands:
    with mp_face_detection.FaceDetection(
    model_selection=0, min_detection_confidence=0.8) as face_detection:
        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                # If loading a video, use 'break' instead of 'continue'.
                continue

            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            results_hand = hands.process(image)
            results_face = face_detection.process(image)
            
            image_height, image_width, _ = image.shape

            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results_hand.multi_hand_landmarks:
                for hand_landmarks in results_hand.multi_hand_landmarks:
                    thumb_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x * image_width
                    thumb_y = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y * image_height
                    # print(
                    #     f'Thumb tip coordinates: (',
                    #     f'{hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x * image_width}, '
                    #     f'{hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y * image_height})'
                    # )
                    mp_drawing.draw_landmarks(
                        image, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                        connection_drawing_spec=con_drawing_spec,
                        landmark_drawing_spec=landmark_drawing_spec)
            if results_face.detections:
                for detection in results_face.detections:
                    cx_min, cy_min, cx_max, cy_max = get_bb(image, detection)
                    if cx_min is not None:
                        dx, dy = (cx_max-cx_min, cy_max-cy_min)
                        cx = cx_min + dx//2
                        cy = cy_min + dy//2
                        cv2.rectangle(image, (cx_min, cy_min), (cx_max, cy_max), (0, 255, 0), 2)
            if thumb_x and thumb_y and cx_min and cy_max:
                print(calculateDistance(thumb_x,thumb_y, cx_max,cy))
            cv2.imshow('MediaPipe Hands', image)
            if cv2.waitKey(5) & 0xFF == 27:
                break
cap.release()