#!/usr/bin/python3
"""
:file: facial_stream.py
:brief: Uses a trained model with OpenCV and MediaPipe to detect emotions
:detail: Based on https://github.com/computervisioneng/emotion-recognition-python-scikit-learn-mediapipe
"""

import pickle
import cv2
from os import listdir
from read_landmarks import get_face_landmarks

DATA_PATH = 'vision/img/dataset/'
EMOTION_MODEL = 'vision/model/facial_emotion_model'

# Get set of emotions from dataset directory
emotions = sorted(listdir(DATA_PATH))
num_of_emotions = len(emotions)
emotions.append('FACE NOT FOUND')

# Retrieve the trained model callable
with open(EMOTION_MODEL, 'rb') as f:
    model = pickle.load(f)

cap = cv2.VideoCapture(0)
while cap.isOpened():
    # Read frame, get facial landmarks, and draw them on the image
    ret, frame = cap.read()
    face_landmarks = get_face_landmarks(frame, draw=True, static_image_mode=False)
    try:
        # Attempt to predict the facial emotion from the given landmarks
        output = model.predict([face_landmarks])
    except ValueError:
        # If face can't be found output the last emotions state (face not found)
        output = [num_of_emotions]

    # Put text displaying the emotion in the top left corner
    cv2.putText(frame, emotions[int(output[0])], (10, 60),
               cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 5)
    cv2.imshow('Emotion Classifier', frame)

    # Press 'q' to break out of loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
