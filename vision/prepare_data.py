#!/usr/bin/python3
"""
:file: prepare_data.py
:brief: Uses mediapipe and scikitlearn to do emotion detection
:detail: Based on https://github.com/computervisioneng/emotion-recognition-python-scikit-learn-mediapipe
"""

import os
import cv2
import numpy as np
from read_landmarks import get_face_landmarks

DATA_PATH = './img/dataset/'


def prepare_data():
    output = []
    for emotion_lbl, emotion in enumerate(sorted(os.listdir(DATA_PATH))):
        for image_name in os.listdir(os.path.join(DATA_PATH, emotion)):
            # Construct current image's path
            image_path = os.path.join(DATA_PATH, emotion, image_name)

            # Read image in and get facial landmarks
            image = cv2.imread(image_path)
            face_landmarks = get_face_landmarks(image)

            # Ensures the all landmarks are present
            if len(face_landmarks) == 1404:
                # Appends index of emotion to end of array
                face_landmarks.append(int(emotion_lbl))
                output.append(face_landmarks)

    # Saves all data to a numpy array in a file
    np.savetxt('data.txt', np.asarray(output))


if __name__ == '__main__':
    prepare_data()
