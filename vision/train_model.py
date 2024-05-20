#!/usr/bin/python3
"""
:file: train_model.py
:brief: Uses scikitlearn to train a model for emotion detection using a pre-
        created dataset and random forest classification
:detail: Based on https://github.com/computervisioneng/emotion-recognition-python-scikit-learn-mediapipe
"""

import pickle
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, confusion_matrix

# Load data from the text file
DATA_FILE = "data.txt"
data = np.loadtxt(DATA_FILE)

# Split data into coordinate landmarks (coords) and emotion labels (lbl)
# Creates a 2D array of all data excluding the last element in each array
coords = data[:, :-1]
# Creates an array containing all labels which are in the last column
lbl = data[:, -1]

# Split the data into training and testing sets
coords_train, coords_test, lbl_train, lbl_test = train_test_split(
    coords,
    lbl,
    test_size=0.2, # Takes a random sample of data (20%)
    random_state=100, # Random reproducible seed value
    shuffle=True, # Shuffles data before splitting
    stratify=lbl # Sets labels
)

rf_classifier = RandomForestClassifier()
# Train the classifier on the sample of training data
rf_classifier.fit(coords_train, lbl_train)

# Make prediction on the test data and evaluate accuracy
lbl_pred = rf_classifier.predict(coords_test)
accuracy = accuracy_score(lbl_test, lbl_pred)
print(f"Accuracy: {accuracy * 100:.2f}%")
print(confusion_matrix(lbl_test, lbl_pred))

# Save callable to a file
with open('./model', 'wb') as f:
    pickle.dump(rf_classifier, f)
