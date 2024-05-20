"""
:file: hand_img.py
:brief: Identifies hand landmarks from a static image using MediaPipe
:detail: Note this implementation uses MediaPipe tasks instead of the more
         concise solutions submodule
:author: Joseph Smith
"""
# https://developers.google.com/mediapipe/solutions/vision/hand_landmarker/python

import cv2
import numpy as np
import mediapipe as mp
from pathlib import Path
from mediapipe.python import solutions
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2

IMAGE_FILES = ['woman_hands.jpg', 'webcam.jpg']
IMAGE_PATH = str(Path(__file__, '..', 'img', IMAGE_FILES[0]).resolve())
MODEL_PATH = str(Path(__file__, '..', 'model', 'hand_landmarker.task').resolve())

base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
options = vision.HandLandmarkerOptions(base_options=base_options,
                                       num_hands=2)
detector = vision.HandLandmarker.create_from_options(options)

image = mp.Image.create_from_file(IMAGE_PATH)
# Image.numpy_view() does not allow for modicication of the returned numpy
#  array so we have to copy it
frame = np.copy(image.numpy_view())

detection_result = detector.detect(image)

for hand in detection_result.hand_landmarks:
    # Insert landmark data into protobuf class form for serialising
    landmarks_pb = [landmark_pb2.NormalizedLandmark(
        x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand]
    landmarks_pb_list = landmark_pb2.NormalizedLandmarkList()
    landmarks_pb_list.landmark.extend(landmarks_pb)

    annotated_image = solutions.drawing_utils.draw_landmarks(
        image=frame,
        landmark_list=landmarks_pb_list,
        connections=solutions.hands.HAND_CONNECTIONS,
        landmark_drawing_spec=solutions.drawing_styles.get_default_hand_landmarks_style(),
        connection_drawing_spec=solutions.drawing_styles.get_default_hand_connections_style(),
    )

cv2.imshow("Hand Test Image", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
