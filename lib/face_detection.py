import cv2
import mediapipe as mp
from helper import *

SHORT_RANGE = True
DETECTION_CONFIDENCE = 0.8

# For webcam input:
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('evan.mp4')
mp_face_detection = mp.solutions.face_detection

with mp_face_detection.FaceDetection(
    model_selection=0 if SHORT_RANGE else 1, 
    min_detection_confidence=DETECTION_CONFIDENCE) as face_detection:
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
    results = face_detection.process(image)

    # Draw the face mesh annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    original_frame_bgr = image.copy()
    if results.detections:
      for detection in results.detections:
        cx_min, cy_min, cx_max, cy_max = get_bb(image, detection)
        if cx_min is not None:
          cv2.rectangle(image, (cx_min, cy_min), (cx_max, cy_max), (0, 255, 0), 2)
    cv2.imshow('Frame', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()
