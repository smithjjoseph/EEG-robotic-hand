#!/usr/bin/python3
"""
:file: read_landmarks.py
:brief: Helper file using mediapipe for getting facial landmarks
:detail: Based on https://github.com/computervisioneng/emotion-recognition-python-scikit-learn-mediapipe
"""

import cv2
import mediapipe as mp


def draw_landmarks(image, results):
    # Draw landmarks on image using mediapipe functionality
    mp_drawing = mp.solutions.drawing_utils
    drawing_spec = mp_drawing.DrawingSpec(thickness=2, circle_radius=1)
    mp_drawing.draw_landmarks(
        image=image,
        landmark_list=results.multi_face_landmarks[0],
        connections=mp.solutions.face_mesh.FACEMESH_CONTOURS,
        landmark_drawing_spec=drawing_spec,
        connection_drawing_spec=drawing_spec)


def get_face_landmarks(image, draw=False, static_image_mode=True):
    # Read the input image and convert colour space
    image_input_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # Create a face landmark identification object using settings
    face_mesh = mp.solutions.face_mesh.FaceMesh(static_image_mode=static_image_mode,
                                                max_num_faces=1,
                                                min_detection_confidence=0.5)
    # Get the landmarks of the given image
    results = face_mesh.process(image_input_rgb)

    image_landmarks = []
    if results.multi_face_landmarks:
        if draw:
            draw_landmarks(image, results)

        single_face_lm = results.multi_face_landmarks[0].landmark
        # Normalise the location of all facial landmarks so that they can be
        # compared to each other despite location in image
        x_lm = []
        y_lm = []
        z_lm = []
        # Append each landmark coordinate in each direction to its own list so
        # that the min can be taken
        for lm_group in single_face_lm:
            x_lm.append(lm_group.x)
            y_lm.append(lm_group.y)
            z_lm.append(lm_group.z)
        min_x = min(x_lm)
        min_y = min(y_lm)
        min_z = min(z_lm)
        # Normalise each coordinate
        for lm_coord in range(len(x_lm)):
            image_landmarks.append(x_lm[lm_coord] - min_x)
            image_landmarks.append(y_lm[lm_coord] - min_y)
            image_landmarks.append(z_lm[lm_coord] - min_z)

    return image_landmarks
